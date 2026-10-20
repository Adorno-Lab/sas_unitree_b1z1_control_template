#python
"""
CoppeliaSim 4.7 Python child script: emulates the SAS driver of the Unitree B1
(sas_robot_driver_unitree_b1, high-level control) with the legged robot driver
standard of sas_tools. A control package that uses LeggedRobotDriverClient and
RobotDriverClient runs unchanged against this scene or against the real robot.

The legs do not step: the base is static and moved by numerical integration. In
WALKING, the twist is integrated and moves the trunk, carrying the whole robot. In
STANDING, the trunk follows the target orientation and height.

Topics, under TOPIC_PREFIX (same names, types, and QoS as the real driver):
    get/info                     sas_legged_msgs/LeggedRobotInfo (transient_local)
    get/status                   sas_legged_msgs/LeggedRobotStatus
    get/imu                      sensor_msgs/Imu (imu_link of the B1)
    get/twist                    geometry_msgs/TwistStamped (measured twist of the base, body frame)
    set/high_level_mode          sas_legged_msgs/HighLevelMode
    set/target_twist             geometry_msgs/TwistStamped (WALKING)
    set/target_base_height       std_msgs/Float64 (STANDING)
    set/target_base_orientation  geometry_msgs/QuaternionStamped (STANDING)
    set/watchdog_trigger         sas_msgs/WatchdogTrigger
    get/joint_states, get/joint_positions_min, get/joint_positions_max (empty)
and SHUTDOWN_TOPIC (/sas/set/shutdown: the launch files of the real driver remap its set/shutdown there, the topic
of the emergency stop of the lab),
and under TOPIC_PREFIX/<limb>, for front_right_leg, front_left_leg, rear_right_leg, rear_left_leg:
    get/joint_states             sensor_msgs/JointState
    get/joint_positions_min      std_msgs/Float64MultiArray
    get/joint_positions_max      std_msgs/Float64MultiArray
    set/target_joint_positions   std_msgs/Float64MultiArray (never applied: the legs are not commandable)
    set/shutdown                 sas_msgs/Bool

Behaviour of the real driver that is emulated:
    - The robot starts in IDLE. Modes: IDLE (zero velocity), STANDING (base orientation and height), and
      WALKING (twist). The legs never accept targets.
    - The measured twist (get/twist) is the velocity of the base in the body frame (vx, vy, wz), the one the
      trunk moves with (the B1 publishes the estimate of its locomotion controller).
    - The twist is saturated to the ranges of the B1: vx [-0.8, 1.2] m/s, vy [-0.25, 0.25] m/s,
      wz [-0.75, 0.75] rad/s. It is zero if no twist arrived for TWIST_TIMEOUT_SEC (twist_timeout_sec).
    - Entering STANDING: the robot stops (zero velocity) until its velocity is below 0.1, and then stands.
      The orientation is relative to the frame F_f, the pose of the trunk when it starts standing. The
      angles are clamped to roll [-0.3, 0.3], pitch [-0.3, 0.3], and yaw [-0.6, 0.6] rad. The height is
      absolute: the B1 takes its difference with 0.58 m, clamped to [-0.16, 0.16] m.
    - Leaving STANDING: the trunk goes back to the neutral pose for 1 s before the robot walks again.
    - Every mode change resets the base orientation and height targets. Targets that arrive in a mode that
      does not accept them are dropped (LeggedRobotDriverROS consumes them anyway).
    - The set/ topics, except set/shutdown, accept a single publisher.
    - The watchdog of SAS: the first trigger enables it, and it trips if the triggers stop for longer than
      the period, if their delay is larger than the maximum acceptable delay, or if the status is false.
    - A shutdown signal, a second publisher, or a tripped watchdog stops the driver: the robot stops (and
      goes back to the neutral pose if it stands), and the topics disappear. The simulation, and the Z1
      driver, keep running, as the B1 driver is a separate process on the real robot.

Differences with the real robot:
    - The legs keep the configuration they have at the start of the simulation, and do not step. Their
      shapes are not respondable, so they do not drag on the floor.
    - Walking: the trunk slides on the plane with the twist (vx, vy, wz in its frame, integrated with unit
      dual quaternions, see TwistIntegrator; requires dqrobotics in CoppeliaSim's Python), with maximum
      accelerations that are assumptions. The robot can pass through objects.
    - Standing: the whole robot rotates and moves with the trunk (there is no inverse kinematics to keep
      the feet on the ground), at the rates STAND_POSE_ANGULAR_RATE and STAND_POSE_HEIGHT_RATE (assumptions).
    - The topics are published once per simulation step (50 ms by default) instead of every 2 ms.
    - The pose of the robot is restored when the simulation stops.

Attach it as a non-threaded Python child script to the model base (UnitreeB1_1), or set ROBOT_BASE_PATH.
Launch CoppeliaSim from a terminal where ROS 2 and the workspace with sas_legged_msgs are sourced.

Scene mode: this script only runs when the environment variable B1Z1_SCENE_MODE is 'simulation' (or not
set) when CoppeliaSim starts.
"""

import math
import os
import time

import rclpy
from dqrobotics import DQ, E_, i_, j_, k_, Ad, exp, normalize, rotation, translation
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import TwistStamped, QuaternionStamped
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64, Float64MultiArray
from sas_msgs.msg import Bool, WatchdogTrigger
from sas_legged_msgs.msg import HighLevelMode, LeggedRobotInfo, LeggedRobotStatus, LimbInfo

# ----------------------------------------------------------------------------
# Configuration
# ----------------------------------------------------------------------------
# Scene mode, chosen with an environment variable when CoppeliaSim is started. Only 'simulation' (the
# simulated drivers run) exists for now.
SCENE_MODE_VARIABLE = 'B1Z1_SCENE_MODE'
SCENE_MODES = ('simulation',)
ACTIVE_IN_MODE = 'simulation'

# Same prefix as the real driver (node b1_1 in the namespace sas_b1), and its remapped shutdown topic.
TOPIC_PREFIX = '/sas_b1/b1_1'
SHUTDOWN_TOPIC = '/sas/set/shutdown'
NODE_NAME = 'coppeliasim_b1_1'

# '..' = the parent of this script object, i.e. the object the script is attached to (in CoppeliaSim 4.7,
# '.' is the script object itself). The trunk and the IMU are found by alias in its tree.
ROBOT_BASE_PATH = '..'
TRUNK_ALIAS = 'trunk_respondable'
IMU_ALIAS = 'imu_link_respondable'

# The limbs of the real driver (sas_robot_driver_unitree_b1.cpp, kB1Legs), in the same order: name, and joint
# names of unitree_ros/robots/b1_description/xacro/b1.urdf (the aliases of the joints in the scene). The joint
# limits (rad) are the same for every leg: unitree_legged_sdk/b1_const.h (identical in b1.urdf).
LIMBS = [
    ('front_right_leg', ['FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint']),
    ('front_left_leg',  ['FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint']),
    ('rear_right_leg',  ['RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint']),
    ('rear_left_leg',   ['RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint']),
]
LEG_LOWER_LIMITS = [-0.75, -1.0, -2.6]
LEG_UPPER_LIMITS = [0.75, 3.5, -0.6]

# The high-level control of the real driver (mode: VelocityControl).
SUPPORTED_MODES = [HighLevelMode.IDLE, HighLevelMode.STANDING, HighLevelMode.WALKING]

# Twist. Ranges of the B1 (the real driver saturates the twist to them).
MIN_FORWARD_VELOCITY = -0.8   # m/s
MAX_FORWARD_VELOCITY = 1.2    # m/s
MAX_SIDE_VELOCITY = 0.25      # m/s
MAX_YAW_RATE = 0.75           # rad/s
# Below these values, a component of the twist is zero. Unknown for the B1: zero (no deadband).
MIN_LINEAR_VELOCITY = 0.0     # m/s
MIN_ANGULAR_VELOCITY = 0.0    # rad/s
# Maximum accelerations of the base. Assumptions (the limits of the real robot are not known).
MAX_LINEAR_ACCELERATION = 1.0     # m/s^2
MAX_ANGULAR_ACCELERATION = 2.0    # rad/s^2
# The target twist is zero if no twist arrived for this long, as twist_timeout_sec in the real driver.
TWIST_TIMEOUT_SEC = 0.2

# Standing (FORCE_STAND), as DriverUnitreeB1 and RobotDriverUnitreeB1.
SPEED_THRESHOLD_TO_STAND = 0.1      # The robot stands once its velocities are below this value.
STAND_EXIT_DURATION = 1.0           # s in the neutral pose before walking again.
BASE_ORIENTATION_LOWER_LIMITS = [-0.3, -0.3, -0.6]  # roll, pitch, yaw (rad), relative to F_f
BASE_ORIENTATION_UPPER_LIMITS = [0.3, 0.3, 0.6]
NOMINAL_BASE_HEIGHT = 0.58          # m. The B1 takes the height as a difference with it.
MAX_BASE_HEIGHT_DELTA = 0.16        # m
# Rates at which the trunk moves toward the target pose. Assumptions (the real robot moves smoothly).
STAND_POSE_ANGULAR_RATE = 0.5       # rad/s, for each angle
STAND_POSE_HEIGHT_RATE = 0.1        # m/s


def find_in_tree(sim, base_handle, aliases, object_type):
    """Finds objects by alias anywhere in the base object's tree."""
    by_alias = {}
    for h in sim.getObjectsInTree(base_handle, object_type, 0):
        by_alias.setdefault(sim.getObjectAlias(h), h)
    missing = [n for n in aliases if n not in by_alias]
    if missing:
        raise RuntimeError(f"objects not found: {missing}. Objects in tree: {sorted(by_alias)}")
    return [by_alias[n] for n in aliases]


def rotation_to_body(m, v):
    """Expresses the world vector v in the frame of the object matrix m (R^T v)."""
    # Rotation from body to world: R[r][c] = m[4*r + c].
    return [sum(m[4 * r + c] * v[r] for r in range(3)) for c in range(3)]


def deadband(value, minimum):
    return 0.0 if abs(value) < minimum else value


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def step_toward(value, target, max_step):
    return value + clamp(target - value, -max_step, max_step)


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


def rpy_to_rotation(roll, pitch, yaw):
    """r = r_z(yaw)*r_y(pitch)*r_x(roll) (ZYX roll-pitch-yaw), as rpy_to_unit_quaternion() of sas_tools."""
    return ((math.cos(yaw / 2) + k_ * math.sin(yaw / 2)) *
            (math.cos(pitch / 2) + j_ * math.sin(pitch / 2)) *
            (math.cos(roll / 2) + i_ * math.sin(roll / 2)))


def rotation_to_rpy(r):
    """ZYX roll-pitch-yaw angles of the unit quaternion r, as unit_quaternion_to_rpy() of sas_tools."""
    x_axis = (r * i_ * r.conj()).vec3()
    y_axis = (r * j_ * r.conj()).vec3()
    z_axis = (r * k_ * r.conj()).vec3()
    roll = math.atan2(y_axis[2], z_axis[2])
    pitch = math.atan2(-x_axis[2], math.sqrt(y_axis[2] ** 2 + z_axis[2] ** 2))
    yaw = math.atan2(x_axis[1], x_axis[0])
    return [roll, pitch, yaw]


class TwistIntegrator:
    """Moves the trunk with the twist: the target twist (body frame) is limited (deadband, ranges of the B1,
    maximum acceleration), expressed in the inertial frame, and integrated into the planar pose of the trunk,
    a unit dual quaternion (Adorno, Robot Kinematic Modeling and Control Based on Dual Quaternion Algebra,
    Part I, Section 2.6.1, eq. 2.6.7):
        twist_u = wz*k_ + E_*(vx*i_ + vy*j_)                     (twist in the body frame)
        uvec_inertial_frame = Ad(xbase, twist_u).vec6()          (twist in the inertial frame)
        xbase = exp(T/2*DQ(uvec_inertial_frame))*xbase
    With the trunk level at the start (its z axis vertical), the planar twist keeps its height, roll, and
    pitch."""

    def __init__(self, xbase):
        self.xbase = xbase
        self.vx = self.vy = self.wz = 0.0       # Current velocity, in the frame of the trunk.

    def speed(self):
        return max(abs(self.vx), abs(self.vy), abs(self.wz))

    def step(self, target, dt):
        """target: (vx, vy, wz) in the frame of the trunk, or None to stop. dt: time step, in s."""
        tvx, tvy, twz = target if target is not None else (0.0, 0.0, 0.0)
        tvx = clamp(deadband(tvx, MIN_LINEAR_VELOCITY), MIN_FORWARD_VELOCITY, MAX_FORWARD_VELOCITY)
        tvy = clamp(deadband(tvy, MIN_LINEAR_VELOCITY), -MAX_SIDE_VELOCITY, MAX_SIDE_VELOCITY)
        twz = clamp(deadband(twz, MIN_ANGULAR_VELOCITY), -MAX_YAW_RATE, MAX_YAW_RATE)
        self.vx = step_toward(self.vx, tvx, MAX_LINEAR_ACCELERATION * dt)
        self.vy = step_toward(self.vy, tvy, MAX_LINEAR_ACCELERATION * dt)
        self.wz = step_toward(self.wz, twz, MAX_ANGULAR_ACCELERATION * dt)
        if self.vx == 0.0 and self.vy == 0.0 and self.wz == 0.0:
            return
        twist_u = self.wz * k_ + E_ * (self.vx * i_ + self.vy * j_)
        uvec_inertial_frame = Ad(self.xbase, twist_u).vec6()
        self.xbase = normalize(exp(dt / 2 * DQ(uvec_inertial_frame)) * self.xbase)


class StandPose:
    """The pose of the trunk relative to its planar pose while standing: roll, pitch, yaw (rad), and the
    height delta (m), moved toward their targets at STAND_POSE_ANGULAR_RATE and STAND_POSE_HEIGHT_RATE."""

    def __init__(self):
        self.current = [0.0, 0.0, 0.0, 0.0]
        self.target = [0.0, 0.0, 0.0, 0.0]

    def is_neutral(self):
        return all(abs(v) < 1e-9 for v in self.current)

    def step(self, dt):
        rates = [STAND_POSE_ANGULAR_RATE] * 3 + [STAND_POSE_HEIGHT_RATE]
        self.current = [step_toward(c, t, rate * dt) for c, t, rate in zip(self.current, self.target, rates)]

    def offset(self):
        """Unit dual quaternion of the trunk relative to its planar pose: raised by the height delta, and
        rotated by r_z(yaw)*r_y(pitch)*r_x(roll)."""
        roll, pitch, yaw, height_delta = self.current
        return (1 + 0.5 * E_ * height_delta * k_) * rpy_to_rotation(roll, pitch, yaw)


class Limb:
    """One leg of the robot: a standard SAS robot driver under TOPIC_PREFIX/<name>."""

    def __init__(self, node, name, joint_names, handles):
        self.name = name
        self.joint_names = joint_names
        self.handles = handles
        prefix = f'{TOPIC_PREFIX}/{name}'
        self.target_topic = f'{prefix}/set/target_joint_positions'
        self.joint_states_pub = node.create_publisher(JointState, f'{prefix}/get/joint_states', 1)
        self.min_pub = node.create_publisher(Float64MultiArray, f'{prefix}/get/joint_positions_min', 1)
        self.max_pub = node.create_publisher(Float64MultiArray, f'{prefix}/get/joint_positions_max', 1)
        node.create_subscription(Float64MultiArray, self.target_topic, self._target_callback, 1)
        node.create_subscription(Bool, f'{prefix}/set/shutdown', node.shutdown_callback, 1)
        self._node = node

    def _target_callback(self, msg):
        # The legs are never commandable: the target is kept by the server of the real driver, but never applied.
        self._node.count_callback()
        self._node.check_exclusive_publisher(self.target_topic)


class UnitreeB1Driver(Node):
    """The topics of the real driver, with the scene as the robot."""

    def __init__(self, sim, base, trunk, imu, limbs):
        super().__init__(NODE_NAME)
        self.sim = sim
        self.base = base
        self.trunk = trunk
        self.imu = imu
        self.mode = HighLevelMode.IDLE
        self.fault = None
        self.info_sent = False
        self.callback_count = 0
        self.dynamics = sim.getBoolParam(sim.boolparam_dynamics_handling_enabled)
        self.gravity = sim.getArrayParam(sim.arrayparam_gravity)
        self.last_linear_velocity = None
        self.target_twist = None        # (vx, vy, wz), body frame, from the last twist received.
        self.target_twist_time = None   # time.time() of its reception.

        # The trunk is moved by integration; the model base (which carries the robot) follows it. The pose of the
        # trunk relative to the base is constant.
        x_base = pose_to_dq(sim.getObjectPose(base, sim.handle_world))
        x_trunk = pose_to_dq(sim.getObjectPose(trunk, sim.handle_world))
        self.x_trunk_in_base = x_base.conj() * x_trunk
        self.twist_integrator = TwistIntegrator(x_trunk)
        self.stand_pose = StandPose()
        self.standing = False           # Stopped and in FORCE_STAND (STANDING), until it leaves the neutral pose.
        self.leave_stand_time = None    # Simulation time when the robot started leaving the stand pose.

        self.watchdog_enabled = False
        self.watchdog_period = None
        self.watchdog_maximum_delay = None
        self.watchdog_status = True
        self.watchdog_time_from_the_client = None
        self.watchdog_time_from_the_server = None

        self.limbs = [Limb(self, name, joint_names, handles) for (name, joint_names), handles in zip(LIMBS, limbs)]
        # The legs hold the configuration they have at the start of the simulation.
        self.leg_configuration = [[sim.getJointPosition(h) for h in limb.handles] for limb in self.limbs]

        p = TOPIC_PREFIX
        self.topic_twist = f'{p}/set/target_twist'
        self.topic_mode = f'{p}/set/high_level_mode'
        self.topic_height = f'{p}/set/target_base_height'
        self.topic_orientation = f'{p}/set/target_base_orientation'
        self.topic_watchdog = f'{p}/set/watchdog_trigger'
        self.create_subscription(TwistStamped, self.topic_twist, self._twist_callback, 1)
        self.create_subscription(HighLevelMode, self.topic_mode, self._mode_callback, 1)
        self.create_subscription(Float64, self.topic_height, self._base_height_callback, 1)
        self.create_subscription(QuaternionStamped, self.topic_orientation, self._base_orientation_callback, 1)
        self.create_subscription(WatchdogTrigger, self.topic_watchdog, self._watchdog_callback, 1)
        self.create_subscription(Bool, SHUTDOWN_TOPIC, self.shutdown_callback, 1)

        info_qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                              reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.info_pub = self.create_publisher(LeggedRobotInfo, f'{p}/get/info', info_qos)
        self.status_pub = self.create_publisher(LeggedRobotStatus, f'{p}/get/status', 1)
        self.imu_pub = self.create_publisher(Imu, f'{p}/get/imu', 1)
        self.measured_twist_pub = self.create_publisher(TwistStamped, f'{p}/get/twist', 1)
        # The base has no joints: its SAS joint topics exist, but they are empty.
        self.base_joint_states_pub = self.create_publisher(JointState, f'{p}/get/joint_states', 1)
        self.base_min_pub = self.create_publisher(Float64MultiArray, f'{p}/get/joint_positions_min', 1)
        self.base_max_pub = self.create_publisher(Float64MultiArray, f'{p}/get/joint_positions_max', 1)

        self.ros_executor = SingleThreadedExecutor()
        self.ros_executor.add_node(self)

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def count_callback(self):
        self.callback_count += 1

    def fail(self, reason):
        """Stops the driver, as an exception or a shutdown does in the real one. The first reason is kept."""
        if self.fault is None:
            self.fault = reason

    def check_exclusive_publisher(self, topic):
        if self.count_publishers(topic) > 1:
            self.fail(f'{topic} must be exclusively published and there is more than one publisher connected.')
            return False
        return True

    def shutdown_callback(self, msg):
        self.count_callback()
        if msg.data:
            self.fail('The shutdown signal was received!')

    def _twist_callback(self, msg):
        self.count_callback()
        if not self.check_exclusive_publisher(self.topic_twist):
            return
        self.target_twist = (msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z)
        self.target_twist_time = time.time()

    def active_twist(self):
        """The twist the trunk follows: the last one received, in WALKING and if not older than
        TWIST_TIMEOUT_SEC; otherwise None (stop)."""
        if (self.mode != HighLevelMode.WALKING or self.target_twist is None
                or time.time() - self.target_twist_time > TWIST_TIMEOUT_SEC):
            return None
        return self.target_twist

    def _mode_callback(self, msg):
        self.count_callback()
        if not self.check_exclusive_publisher(self.topic_mode):
            return
        if msg.mode == self.mode:
            return
        if msg.mode not in SUPPORTED_MODES:
            self.get_logger().warn(f'::Mode change rejected: the mode {msg.mode} is not supported.')
            return
        self.mode = msg.mode
        self.get_logger().info(f'::High-level mode changed to {self.mode}')
        # As RobotDriverUnitreeB1: every mode change stops the robot and resets the target pose.
        self.target_twist = None
        self.stand_pose.target = [0.0, 0.0, 0.0, 0.0]

    def _base_height_callback(self, msg):
        self.count_callback()
        if not self.check_exclusive_publisher(self.topic_height):
            return
        if self.mode != HighLevelMode.STANDING or not math.isfinite(msg.data):
            return  # Not accepted in this mode (or reset to the nominal height by the real driver).
        self.stand_pose.target[3] = clamp(msg.data - NOMINAL_BASE_HEIGHT, -MAX_BASE_HEIGHT_DELTA, MAX_BASE_HEIGHT_DELTA)

    def _base_orientation_callback(self, msg):
        self.count_callback()
        if not self.check_exclusive_publisher(self.topic_orientation):
            return
        if self.mode != HighLevelMode.STANDING:
            return  # Not accepted in this mode.
        q = msg.quaternion
        r = DQ([q.w, q.x, q.y, q.z])
        if abs(r.vec4().dot(r.vec4()) - 1.0) > 1e-6:
            self.stand_pose.target[0:3] = [0.0, 0.0, 0.0]  # Not a unit quaternion: reset to neutral, as the real driver.
            return
        rpy = rotation_to_rpy(r)
        clamped = [clamp(a, lo, up) for a, lo, up in zip(rpy, BASE_ORIENTATION_LOWER_LIMITS, BASE_ORIENTATION_UPPER_LIMITS)]
        if clamped != rpy:
            self.get_logger().warn('::The target base orientation was clamped to the limits of the robot.',
                                   throttle_duration_sec=1.0)
        self.stand_pose.target[0:3] = clamped

    def _watchdog_callback(self, msg):
        self.count_callback()
        if not self.check_exclusive_publisher(self.topic_watchdog):
            return
        if not self.watchdog_enabled:
            self.watchdog_enabled = True
            self.watchdog_period = msg.period_in_seconds
            self.watchdog_maximum_delay = msg.maximum_acceptable_delay_in_seconds
            self.get_logger().info(f'::Watchdog initialized with a {self.watchdog_period} second period '
                                   f'and a maximum acceptable delay of {self.watchdog_maximum_delay} seconds')
        elif (abs(msg.period_in_seconds - self.watchdog_period) > 1e-12 or
              abs(msg.maximum_acceptable_delay_in_seconds - self.watchdog_maximum_delay) > 1e-12):
            self.fail('The watchdog period or maximum acceptable delay changed.')
        self.watchdog_status = msg.status
        self.watchdog_time_from_the_client = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        self.watchdog_time_from_the_server = time.time()

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------
    def spin(self, max_callbacks=100):
        """Processes the pending messages (spin_once handles one callback per call)."""
        for _ in range(max_callbacks):
            before = self.callback_count
            self.ros_executor.spin_once(timeout_sec=0.0)
            if self.callback_count == before:
                return

    def check_watchdog(self):
        if not self.watchdog_enabled:
            return
        now = time.time()
        elapsed_time = now - self.watchdog_time_from_the_client
        elapsed_time_same_clock = now - self.watchdog_time_from_the_server
        clock_difference = abs(elapsed_time - elapsed_time_same_clock)
        if clock_difference > self.watchdog_maximum_delay:
            self.fail('The watchdog signal is delayed, or the clocks between the client and server are out of '
                      f'synch! Watchdog signal delay: {1000 * clock_difference:.1f}ms.')
        elif elapsed_time_same_clock > self.watchdog_period:
            self.fail(f'The watchdog signal was lost! The elapsed time was {elapsed_time_same_clock:.3f} seconds, '
                      f'but the period is {self.watchdog_period}.')
        elif not self.watchdog_status:
            self.fail('The watchdog status is false!')

    def set_joint(self, handle, q):
        if self.dynamics and self.sim.getJointMode(handle)[0] == self.sim.jointmode_dynamic:
            self.sim.setJointTargetPosition(handle, q)
        else:
            self.sim.setJointPosition(handle, q)

    def actuation(self):
        """One tick of DriverUnitreeB1 (velocity control) and of the motion of the trunk. After a fault, the robot
        stops as in deinitialize(): it goes back to the neutral pose if it stands, and its velocity goes to zero."""
        if self.fault is None:
            self.spin()
            self.check_watchdog()
        if self.fault is None and not self.info_sent:
            self.send_info()
            self.info_sent = True

        dt = self.sim.getSimulationTimeStep()
        stand = self.fault is None and self.mode == HighLevelMode.STANDING
        if stand:
            self.leave_stand_time = None
            if not self.standing:
                # Stop first: the robot stands once its velocity is below the threshold.
                self.twist_integrator.step(None, dt)
                if self.twist_integrator.speed() < SPEED_THRESHOLD_TO_STAND:
                    self.twist_integrator.vx = self.twist_integrator.vy = self.twist_integrator.wz = 0.0
                    self.standing = True
                    self.get_logger().info('::The robot is stopped. FORCE_STAND.')
        elif self.standing:
            # Leaving the stand pose: neutral pose for STAND_EXIT_DURATION, then walking again.
            self.stand_pose.target = [0.0, 0.0, 0.0, 0.0]
            now = self.sim.getSimulationTime()
            if self.leave_stand_time is None:
                self.leave_stand_time = now
            if now - self.leave_stand_time >= STAND_EXIT_DURATION and self.stand_pose.is_neutral():
                self.standing = False
                self.leave_stand_time = None
        else:
            twist = self.active_twist() if self.fault is None else None
            self.twist_integrator.step(twist, dt)

        if self.standing:
            self.stand_pose.step(dt)
        x_trunk = self.twist_integrator.xbase * self.stand_pose.offset()
        self.sim.setObjectPose(self.base, dq_to_pose(x_trunk * self.x_trunk_in_base.conj()), self.sim.handle_world)

        for limb, configuration in zip(self.limbs, self.leg_configuration):
            for handle, q in zip(limb.handles, configuration):
                self.set_joint(handle, q)

    def sensing(self):
        if self.fault is not None:
            return
        sim = self.sim
        stamp = self.get_clock().now().to_msg()

        for limb in self.limbs:
            msg = JointState()
            msg.header.stamp = stamp
            msg.position = [sim.getJointPosition(h) for h in limb.handles]
            msg.velocity = [sim.getJointVelocity(h) for h in limb.handles]
            msg.effort = ([sim.getJointForce(h) or 0.0 for h in limb.handles] if self.dynamics
                          else [0.0] * len(limb.handles))
            limb.joint_states_pub.publish(msg)
            limb.min_pub.publish(Float64MultiArray(data=LEG_LOWER_LIMITS))
            limb.max_pub.publish(Float64MultiArray(data=LEG_UPPER_LIMITS))

        base_joint_states = JointState()
        base_joint_states.header.stamp = stamp
        self.base_joint_states_pub.publish(base_joint_states)
        self.base_min_pub.publish(Float64MultiArray())
        self.base_max_pub.publish(Float64MultiArray())

        self.send_imu()

        measured_twist = TwistStamped()
        measured_twist.header.stamp = stamp
        measured_twist.twist.linear.x = self.twist_integrator.vx
        measured_twist.twist.linear.y = self.twist_integrator.vy
        measured_twist.twist.angular.z = self.twist_integrator.wz
        self.measured_twist_pub.publish(measured_twist)

        status = LeggedRobotStatus()
        status.header.stamp = stamp
        status.mode = self.mode
        status.accepts_twist = self.mode == HighLevelMode.WALKING
        status.accepts_base_orientation = self.mode == HighLevelMode.STANDING
        status.accepts_base_height = self.mode == HighLevelMode.STANDING
        status.commandable_limbs = [False] * len(self.limbs)
        self.status_pub.publish(status)

    def send_info(self):
        info = LeggedRobotInfo()
        info.limbs = [LimbInfo(name=limb.name, joint_names=limb.joint_names) for limb in self.limbs]
        info.supported_modes = SUPPORTED_MODES
        info.supports_twist = True
        info.supports_base_height = True
        info.supports_base_orientation = True
        info.supports_base_height_while_walking = False
        info.supports_measured_twist = True
        info.min_base_roll, info.min_base_pitch, info.min_base_yaw = BASE_ORIENTATION_LOWER_LIMITS
        info.max_base_roll, info.max_base_pitch, info.max_base_yaw = BASE_ORIENTATION_UPPER_LIMITS
        info.min_base_height = NOMINAL_BASE_HEIGHT - MAX_BASE_HEIGHT_DELTA
        info.max_base_height = NOMINAL_BASE_HEIGHT + MAX_BASE_HEIGHT_DELTA
        info.nominal_base_height = NOMINAL_BASE_HEIGHT
        self.info_pub.publish(info)

    def send_imu(self):
        """Orientation, angular velocity, and linear acceleration (with gravity) of the IMU, as the real IMU.
        The velocities are computed from the motion of the base, which is not dynamic."""
        sim = self.sim
        x, y, z, w = sim.getObjectQuaternion(self.imu, sim.handle_world)
        linear_velocity, angular_velocity = sim.getObjectVelocity(self.imu)
        m = sim.getObjectMatrix(self.imu, sim.handle_world)

        if self.last_linear_velocity is None:
            acceleration = [0.0, 0.0, 0.0]
        else:
            dt = sim.getSimulationTimeStep()
            acceleration = [(v - v0) / dt for v, v0 in zip(linear_velocity, self.last_linear_velocity)]
        self.last_linear_velocity = linear_velocity
        # An accelerometer measures the acceleration minus gravity: +9.81 on z when the robot is upright and still.
        specific_force = [a - g for a, g in zip(acceleration, self.gravity)]

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z = w, x, y, z
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = rotation_to_body(m, angular_velocity)
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = \
            rotation_to_body(m, specific_force)
        self.imu_pub.publish(msg)


def check_joint_limits(sim, limbs):
    """Warns if the joint limits of the scene differ from the ones published by the driver."""
    for (name, joint_names), handles in zip(LIMBS, limbs):
        for joint_name, h, lo, up in zip(joint_names, handles, LEG_LOWER_LIMITS, LEG_UPPER_LIMITS):
            _, (scene_lo, scene_range) = sim.getJointInterval(h)
            if abs(scene_lo - lo) > 1e-3 or abs(scene_lo + scene_range - up) > 1e-3:
                print(f"Warning: {name}/{joint_name}: the scene limits [{scene_lo:.4f}, {scene_lo + scene_range:.4f}] "
                      f"differ from the driver limits [{lo:.4f}, {up:.4f}].")


def scene_mode():
    """The mode of the scene: the value of SCENE_MODE_VARIABLE, or 'simulation' if it is not set."""
    return os.environ.get(SCENE_MODE_VARIABLE, 'simulation').strip().lower()


def sysCall_init():
    sim = require('sim')
    self.node = None
    self.ros_initialized = False
    self.fault_reported = False
    self.initial_pose = None
    mode = scene_mode()
    if mode != ACTIVE_IN_MODE:
        if mode not in SCENE_MODES:
            print(f"Error: {SCENE_MODE_VARIABLE}='{mode}' is not one of {SCENE_MODES}. The Unitree B1 driver is idle.")
        else:
            print(f"Unitree B1 driver: idle ({SCENE_MODE_VARIABLE}={mode}).")
        return
    try:
        base = sim.getObject(ROBOT_BASE_PATH)
        trunk, = find_in_tree(sim, base, [TRUNK_ALIAS], sim.object_shape_type)
        imu, = find_in_tree(sim, base, [IMU_ALIAS], sim.object_shape_type)
        limbs = [find_in_tree(sim, base, joint_names, sim.object_joint_type) for _, joint_names in LIMBS]
        check_joint_limits(sim, limbs)
        self.initial_pose = (base, sim.getObjectPose(base, sim.handle_world))

        if not rclpy.ok():
            rclpy.init()
            self.ros_initialized = True
        self.node = UnitreeB1Driver(sim, base, trunk, imu, limbs)

        print(f"Unitree B1 driver on '{TOPIC_PREFIX}' (base='{sim.getObjectAlias(base, 1)}', "
              f"limbs={[limb[0] for limb in LIMBS]}). Mode: IDLE.")
    except Exception as e:
        print(f"Error initializing the Unitree B1 driver script: {e}")


def sysCall_actuation():
    if self.node is None:
        return
    self.node.actuation()
    if self.node.fault is not None and not self.fault_reported:
        print(f"Unitree B1 driver stopped: {self.node.fault}")
        self.fault_reported = True
        # The topics disappear, but actuation() keeps stopping the robot in the scene.
        self.node.ros_executor.shutdown()
        self.node.destroy_node()
        self.node.destroyed = True


def sysCall_sensing():
    if self.node is None or getattr(self.node, 'destroyed', False):
        return
    self.node.sensing()


def sysCall_cleanup():
    sim = require('sim')
    if self.node is not None and not getattr(self.node, 'destroyed', False):
        self.node.ros_executor.shutdown()
        self.node.destroy_node()
    self.node = None
    if self.initial_pose is not None:
        base, pose = self.initial_pose
        sim.setObjectPose(base, pose, sim.handle_world)
    if self.ros_initialized:
        rclpy.shutdown()
        self.ros_initialized = False
    print("ROS 2 cleaned up")
