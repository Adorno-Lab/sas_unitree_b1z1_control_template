#python
"""
CoppeliaSim 4.7 Python child script: mirrors the real Unitree B1Z1 in the scene, to use CoppeliaSim as a
visualization tool while the real robot runs. It is the counterpart of sas_unitree_b1_driver.py,
sas_unitree_z1_driver.py, and b1_ground_truth_pose.py, which are idle in this mode.

The script only SUBSCRIBES to the topics of the real robot. It never publishes on them:
    <B1 prefix>/get/ekf/robot_pose           geometry_msgs/PoseStamped (pose of the trunk, from the EKF)
    <B1 prefix>/<leg>/get/joint_states       sensor_msgs/JointState (front_right_leg, front_left_leg, ...)
    <Z1 prefix>/get/joint_states             sensor_msgs/JointState (6 joints and the gripper)
The latest values are set on the scene once per simulation step. Without the EKF, the robot stays where it
is in the scene. The prefixes are /sas_b1/b1_<id> and /sas_z1/z1_<id>, with <id> the value of the
environment variable B1Z1_MIRROR_ROBOT_ID (default 1, the white B1Z1; 2 is the black one).

The dynamics is disabled, so that the physics does not fight the mirrored positions. The pose of the robot and
the joints are restored when the simulation stops. The frames of the scene (sas_object_script: x1, xd1) are not
affected.

Attach it as a non-threaded Python child script to the model base (UnitreeB1_1), or set ROBOT_BASE_PATH.
Launch CoppeliaSim from a terminal where ROS 2 is sourced, with the ROS_DOMAIN_ID of the real robot.

Scene mode: this script only runs when the environment variable B1Z1_SCENE_MODE is 'mirror' when
CoppeliaSim starts (B1Z1_SCENE_MODE=mirror ./coppeliaSim.sh <scene>).
"""

import os
import time

import rclpy
from dqrobotics import DQ, E_, i_, j_, k_, rotation, translation
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

# ----------------------------------------------------------------------------
# Configuration
# ----------------------------------------------------------------------------
SCENE_MODE_VARIABLE = 'B1Z1_SCENE_MODE'
SCENE_MODES = ('simulation', 'mirror')
ACTIVE_IN_MODE = 'mirror'

ROBOT_ID_VARIABLE = 'B1Z1_MIRROR_ROBOT_ID'
NODE_NAME = 'coppeliasim_b1z1_mirror'

# '..' = the parent of this script object, i.e. the object the script is attached to.
ROBOT_BASE_PATH = '..'
TRUNK_ALIAS = 'trunk_respondable'

# The limbs of the real B1 driver, with its joint names (the same in the scene).
B1_LIMBS = [
    ('front_right_leg', ['FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint']),
    ('front_left_leg',  ['FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint']),
    ('rear_right_leg',  ['RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint']),
    ('rear_left_leg',   ['RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint']),
]
Z1_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint_gripper']

# A warning is printed when nothing arrives for this long (e.g. the driver stopped).
STALE_AFTER_SEC = 1.0

# Best effort is compatible with reliable and best-effort publishers; only the latest state is used.
QOS = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.BEST_EFFORT)


def find_in_tree(sim, base_handle, aliases, object_type):
    """Finds objects by alias anywhere in the base object's tree."""
    by_alias = {}
    for h in sim.getObjectsInTree(base_handle, object_type, 0):
        by_alias.setdefault(sim.getObjectAlias(h), h)
    missing = [n for n in aliases if n not in by_alias]
    if missing:
        raise RuntimeError(f"objects not found: {missing}. Objects in tree: {sorted(by_alias)}")
    return [by_alias[n] for n in aliases]


def pose_to_dq(pose):
    """CoppeliaSim pose [x, y, z, qx, qy, qz, qw] -> unit dual quaternion."""
    x, y, z, qx, qy, qz, qw = pose
    r = DQ([qw, qx, qy, qz])
    return r + 0.5 * E_ * (x * i_ + y * j_ + z * k_) * r


def dq_to_pose(x):
    """Unit dual quaternion -> CoppeliaSim pose [x, y, z, qx, qy, qz, qw]."""
    p = translation(x).vec3()
    w, qx, qy, qz = rotation(x).vec4()
    return [p[0], p[1], p[2], qx, qy, qz, w]


class JointMirror:
    """The latest measured positions of one driver, and the scene joints they are set on."""

    def __init__(self, node, topic, handles):
        self.topic = topic
        self.handles = handles
        self.positions = None
        self.time = None
        self.size_warned = False
        self._node = node
        node.create_subscription(JointState, topic, self._callback, QOS)

    def _callback(self, msg):
        self._node.callback_count += 1
        self.positions = list(msg.position)
        self.time = time.time()

    def apply(self, sim):
        if self.positions is None:
            return
        if len(self.positions) != len(self.handles):
            if not self.size_warned:
                print(f"Warning: '{self.topic}' has {len(self.positions)} positions, but the scene has "
                      f"{len(self.handles)} joints. Ignored.")
                self.size_warned = True
            return
        for handle, q in zip(self.handles, self.positions):
            sim.setJointPosition(handle, q)


class UnitreeB1Z1Mirror(Node):
    """Subscribes to the state of the real B1Z1 and sets it on the scene."""

    def __init__(self, sim, base, trunk, b1_limbs, z1_joints, b1_prefix, z1_prefix):
        super().__init__(NODE_NAME)
        self.sim = sim
        self.base = base
        self.callback_count = 0
        # The EKF gives the pose of the trunk; the base is moved, so that the whole robot follows.
        self.x_trunk_in_base = pose_to_dq(sim.getObjectPose(base, sim.handle_world)).conj() * \
            pose_to_dq(sim.getObjectPose(trunk, sim.handle_world))
        self.x_trunk = None
        self.pose_topic = f'{b1_prefix}/get/ekf/robot_pose'
        self.create_subscription(PoseStamped, self.pose_topic, self._pose_callback, QOS)
        self.mirrors = [JointMirror(self, f'{b1_prefix}/{name}/get/joint_states', handles)
                        for (name, _), handles in zip(B1_LIMBS, b1_limbs)]
        self.mirrors.append(JointMirror(self, f'{z1_prefix}/get/joint_states', z1_joints))
        self.state = 'waiting'   # 'waiting' (nothing received yet), 'receiving', or 'stale'
        self.start_time = time.time()
        self.ros_executor = SingleThreadedExecutor()
        self.ros_executor.add_node(self)

    def _pose_callback(self, msg):
        self.callback_count += 1
        if self.x_trunk is None:
            print(f"Unitree B1Z1 mirror: receiving the pose of the robot on '{self.pose_topic}'.")
        p, q = msg.pose.position, msg.pose.orientation
        self.x_trunk = pose_to_dq([p.x, p.y, p.z, q.x, q.y, q.z, q.w])

    def spin(self, max_callbacks=100):
        """Processes the pending messages (spin_once handles one callback per call)."""
        for _ in range(max_callbacks):
            before = self.callback_count
            self.ros_executor.spin_once(timeout_sec=0.0)
            if self.callback_count == before:
                return

    def apply(self):
        if self.x_trunk is not None:
            self.sim.setObjectPose(self.base, dq_to_pose(self.x_trunk * self.x_trunk_in_base.conj()),
                                   self.sim.handle_world)
        for mirror in self.mirrors:
            mirror.apply(self.sim)

    def check_stale(self):
        """Prints when the joint states start arriving, and when they stop for STALE_AFTER_SEC."""
        times = [mirror.time for mirror in self.mirrors if mirror.time is not None]
        reference = max(times) if times else self.start_time
        state = 'receiving' if times and time.time() - reference < STALE_AFTER_SEC else \
                'stale' if time.time() - reference >= STALE_AFTER_SEC else self.state
        if state != self.state:
            if state == 'receiving':
                print("Unitree B1Z1 mirror: receiving the joint states.")
            else:
                print(f"Unitree B1Z1 mirror: no joint state for more than {STALE_AFTER_SEC} s. "
                      f"Are the drivers running, with the same ROS_DOMAIN_ID?")
            self.state = state


def scene_mode():
    """The mode of the scene: the value of SCENE_MODE_VARIABLE, or 'simulation' if it is not set."""
    return os.environ.get(SCENE_MODE_VARIABLE, 'simulation').strip().lower()


def sysCall_init():
    sim = require('sim')
    self.node = None
    self.ros_initialized = False
    self.dynamics_to_restore = None   # restored in sysCall_cleanup(), so that saving the scene keeps the dynamics
    self.initial_pose = None
    self.initial_joint_positions = []
    mode = scene_mode()
    if mode != ACTIVE_IN_MODE:
        if mode not in SCENE_MODES:
            print(f"Error: {SCENE_MODE_VARIABLE}='{mode}' is not one of {SCENE_MODES}. The Unitree B1Z1 mirror is idle.")
        else:
            print(f"Unitree B1Z1 mirror: idle ({SCENE_MODE_VARIABLE}={mode}).")
        return
    try:
        robot_id = os.environ.get(ROBOT_ID_VARIABLE, '1').strip()
        b1_prefix = f'/sas_b1/b1_{robot_id}'
        z1_prefix = f'/sas_z1/z1_{robot_id}'
        base = sim.getObject(ROBOT_BASE_PATH)
        trunk, = find_in_tree(sim, base, [TRUNK_ALIAS], sim.object_shape_type)
        b1_limbs = [find_in_tree(sim, base, joint_names, sim.object_joint_type) for _, joint_names in B1_LIMBS]
        z1_joints = find_in_tree(sim, base, Z1_JOINTS, sim.object_joint_type)
        self.initial_pose = (base, sim.getObjectPose(base, sim.handle_world))
        self.initial_joint_positions = [(h, sim.getJointPosition(h))
                                        for h in [h for limb in b1_limbs for h in limb] + z1_joints]
        if sim.getBoolParam(sim.boolparam_dynamics_handling_enabled):
            self.dynamics_to_restore = True
            sim.setBoolParam(sim.boolparam_dynamics_handling_enabled, False)

        if not rclpy.ok():
            rclpy.init()
            self.ros_initialized = True
        self.node = UnitreeB1Z1Mirror(sim, base, trunk, b1_limbs, z1_joints, b1_prefix, z1_prefix)

        print(f"Unitree B1Z1 mirror on '{b1_prefix}' and '{z1_prefix}' (base='{sim.getObjectAlias(base, 1)}').")
    except Exception as e:
        print(f"Error initializing the Unitree B1Z1 mirror script: {e}")


def sysCall_actuation():
    if self.node is None:
        return
    self.node.spin()
    self.node.apply()
    self.node.check_stale()


def sysCall_cleanup():
    sim = require('sim')
    if self.dynamics_to_restore is not None:
        sim.setBoolParam(sim.boolparam_dynamics_handling_enabled, self.dynamics_to_restore)
        self.dynamics_to_restore = None
    if self.node is not None:
        self.node.ros_executor.shutdown()
        self.node.destroy_node()
        self.node = None
    if self.initial_pose is not None:
        base, pose = self.initial_pose
        sim.setObjectPose(base, pose, sim.handle_world)
        self.initial_pose = None
    for handle, q in self.initial_joint_positions:
        sim.setJointPosition(handle, q)
    self.initial_joint_positions = []
    if self.ros_initialized:
        rclpy.shutdown()
        self.ros_initialized = False
    print("ROS 2 cleaned up")
