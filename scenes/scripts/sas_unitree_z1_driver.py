#python
"""
CoppeliaSim 4.7 Python child script: emulates the SAS driver of the Unitree Z1 arm with its gripper
(sas_robot_driver_unitree_z1), a standard SAS robot driver. A control package that uses RobotDriverClient
runs unchanged against this scene or against the real arm.

Topics, under TOPIC_PREFIX (same names, types, and QoS as the real driver):
    get/joint_states               sensor_msgs/JointState (7 joints: joint1..joint6, and the gripper)
    get/joint_positions_min        std_msgs/Float64MultiArray (7 values, rad)
    get/joint_positions_max        std_msgs/Float64MultiArray (7 values, rad)
    set/target_joint_positions     std_msgs/Float64MultiArray (7 values, rad)
    set/target_joint_velocities    std_msgs/Float64MultiArray (7 values, rad/s; accepted, see below)
    set/watchdog_trigger           sas_msgs/WatchdogTrigger
and SHUTDOWN_TOPIC (/sas/set/shutdown: the launch files of the real driver remap its set/shutdown there).

Behaviour of the real driver that is emulated (with the parameters of its launch files):
    - When the driver starts, the arm moves to INITIAL_CONFIGURATION_DEG (move_robot_to_initial_configuration)
      with OPEN_LOOP_JOINT_CONTROL_GAIN, before the topics are published. Then it holds its configuration
      until a target arrives.
    - MODE 'PositionControl': the arm follows the target with the first-order controller of DriverUnitreeZ1
      (u = -2*gain*(q - q_target), gain 5, limited to keep q within the joint limits and |u| below 3.1416
      rad/s), integrated at the simulation time step. MODE 'RawPositionControl': the target, clamped to the
      joint limits, is applied directly (the kinematic controller is in charge of smooth targets).
    - The gripper (7th value) follows its target, clamped to its limits, with a first-order controller
      (gain 1), in both modes.
    - Joint limits (joint_limits_min/max of the launch files, degrees): they are published, and the commands
      are clamped to them.
    - The set/ topics, except the shutdown, accept a single publisher.
    - The watchdog of SAS: the first trigger enables it, and it trips if the triggers stop for longer than the
      period, if their delay is larger than the maximum acceptable delay, or if the status is false.
    - Messages sent before the arm reaches the initial configuration are lost (the real driver does not serve
      its topics yet).
    - A shutdown signal, a second publisher, a target that does not have 7 values, or a tripped watchdog stops
      the driver: the arm moves back to the configuration it had when the driver started, and the topics
      disappear. The simulation, and the B1 driver, keep running, as the Z1 driver is a separate process.

Differences with the real arm:
    - The joints receive their commands as target positions (dynamic mode), or are set directly.
    - The target velocities are accepted but not used (the real driver sends them as feedforward in
      RawPositionControl).
    - The topics are published once per simulation step (50 ms by default) instead of every 2 ms.

Attach it as a non-threaded Python child script to the base of the Z1 model (UnitreeZ1), or set
ROBOT_BASE_PATH. Launch CoppeliaSim from a terminal where ROS 2 is sourced.

Scene mode: this script only runs when the environment variable B1Z1_SCENE_MODE is 'simulation' (or not
set) when CoppeliaSim starts.
"""

import math
import os
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from sas_msgs.msg import Bool, WatchdogTrigger

# ----------------------------------------------------------------------------
# Configuration
# ----------------------------------------------------------------------------
SCENE_MODE_VARIABLE = 'B1Z1_SCENE_MODE'
SCENE_MODES = ('simulation',)
ACTIVE_IN_MODE = 'simulation'

# Same prefix as the real driver (node z1_1 in the namespace sas_z1), and its remapped shutdown topic.
TOPIC_PREFIX = '/sas_z1/z1_1'
SHUTDOWN_TOPIC = '/sas/set/shutdown'
NODE_NAME = 'coppeliasim_z1_1'

# '..' = the parent of this script object, i.e. the object the script is attached to.
ROBOT_BASE_PATH = '..'
JOINT_ALIASES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint_gripper']

# Parameters of the launch files of the real driver (real_z1_*_robot_launch.py).
MODE = 'RawPositionControl'                    # 'PositionControl' or 'RawPositionControl'
MOVE_ROBOT_TO_INITIAL_CONFIGURATION = True
INITIAL_CONFIGURATION_DEG = [0.0, 90.0, -60.0, -30.0, 0.0, -90.0]
OPEN_LOOP_JOINT_CONTROL_GAIN = 0.4
JOINT_LIMITS_MIN_DEG = [-150.0, 0.0, -160.0, -80.0, -80.0, -160.0, -40.0]
JOINT_LIMITS_MAX_DEG = [150.0, 180.0, 0.0, 80.0, 80.0, 160.0, 0.0]

# Constants of DriverUnitreeZ1.
POSITION_CONTROL_GAIN = 5.0
MAX_JOINT_VELOCITY = 3.1416      # rad/s
GRIPPER_GAIN = 1.0
INITIAL_MOVE_ERROR_NORM = 0.01   # rad
NUMBER_OF_ARM_JOINTS = 6


def find_joints(sim, base_handle, names):
    """Finds joints by alias anywhere in the base object's tree."""
    by_alias = {}
    for h in sim.getObjectsInTree(base_handle, sim.object_joint_type, 0):
        by_alias.setdefault(sim.getObjectAlias(h), h)
    missing = [n for n in names if n not in by_alias]
    if missing:
        raise RuntimeError(f"joints not found: {missing}. Joints in tree: {sorted(by_alias)}")
    return [by_alias[n] for n in names]


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


class UnitreeZ1Driver(Node):
    """The topics of the real driver, with the scene as the arm."""

    def __init__(self, sim, joints):
        super().__init__(NODE_NAME)
        self.sim = sim
        self.joints = joints
        self.dynamics = sim.getBoolParam(sim.boolparam_dynamics_handling_enabled)
        self.fault = None
        self.callback_count = 0
        self.q_min = [math.radians(v) for v in JOINT_LIMITS_MIN_DEG]
        self.q_max = [math.radians(v) for v in JOINT_LIMITS_MAX_DEG]

        # The commanded configuration (numerical integration of DriverUnitreeZ1), from the measured one.
        self.q_ni = [sim.getJointPosition(h) for h in joints]
        self.q_start = list(self.q_ni)      # Configuration when the driver started (where it goes back to).
        self.target = None                  # Last target received (7 values), kept as in RobotDriverServer.
        # 'initializing' (move to the initial configuration, no topics), 'running', or 'stopping' (move back).
        self.state = 'initializing' if MOVE_ROBOT_TO_INITIAL_CONFIGURATION else 'running'
        self.initial_configuration = [math.radians(v) for v in INITIAL_CONFIGURATION_DEG] + [self.q_ni[-1]]

        self.watchdog_enabled = False
        self.watchdog_period = None
        self.watchdog_maximum_delay = None
        self.watchdog_status = True
        self.watchdog_time_from_the_client = None
        self.watchdog_time_from_the_server = None

        p = TOPIC_PREFIX
        self.topic_target = f'{p}/set/target_joint_positions'
        self.topic_target_velocities = f'{p}/set/target_joint_velocities'
        self.topic_watchdog = f'{p}/set/watchdog_trigger'
        self.create_subscription(Float64MultiArray, self.topic_target, self._target_callback, 1)
        self.create_subscription(Float64MultiArray, self.topic_target_velocities, self._target_velocities_callback, 1)
        self.create_subscription(WatchdogTrigger, self.topic_watchdog, self._watchdog_callback, 1)
        self.create_subscription(Bool, SHUTDOWN_TOPIC, self._shutdown_callback, 1)

        self.joint_states_pub = self.create_publisher(JointState, f'{p}/get/joint_states', 1)
        self.min_pub = self.create_publisher(Float64MultiArray, f'{p}/get/joint_positions_min', 1)
        self.max_pub = self.create_publisher(Float64MultiArray, f'{p}/get/joint_positions_max', 1)

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

    def _shutdown_callback(self, msg):
        self.count_callback()
        if msg.data:
            self.fail('The shutdown signal was received!')

    def _target_callback(self, msg):
        self.count_callback()
        if not self.check_exclusive_publisher(self.topic_target):
            return
        if len(msg.data) != len(self.joints):
            self.fail(f'{self.topic_target}: expected a vector of size {len(self.joints)}, got size {len(msg.data)}.')
            return
        self.target = list(msg.data)

    def _target_velocities_callback(self, msg):
        self.count_callback()
        if not self.check_exclusive_publisher(self.topic_target_velocities):
            return
        if len(msg.data) != len(self.joints):
            self.fail(f'{self.topic_target_velocities}: expected a vector of size {len(self.joints)}, '
                      f'got size {len(msg.data)}.')

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

    def first_order_step(self, q_target, gain, dt):
        """One step of the controller of DriverUnitreeZ1 for the arm joints: u = -2*gain*(q - q_target), within
        -gain*(q - q_min) <= u <= gain*(q_max - q) and |u| <= MAX_JOINT_VELOCITY (the QP of _compute_control_inputs,
        whose cost matrix is the identity, so it is solved joint by joint), integrated with dt."""
        for i in range(NUMBER_OF_ARM_JOINTS):
            q = self.q_ni[i]
            lower = max(-gain * (q - self.q_min[i]), -MAX_JOINT_VELOCITY)
            upper = min(gain * (self.q_max[i] - q), MAX_JOINT_VELOCITY)
            u = clamp(-2.0 * gain * (q - q_target[i]), lower, upper)
            self.q_ni[i] = q + dt * u

    def gripper_step(self, target, dt):
        target = clamp(target, self.q_min[-1], self.q_max[-1])
        self.q_ni[-1] += dt * (-GRIPPER_GAIN * (self.q_ni[-1] - target))

    def arm_error_norm(self, q_target):
        return math.sqrt(sum((q - t) ** 2 for q, t in zip(self.q_ni[:NUMBER_OF_ARM_JOINTS], q_target)))

    def set_joint(self, handle, q):
        if self.dynamics and self.sim.getJointMode(handle)[0] == self.sim.jointmode_dynamic:
            self.sim.setJointTargetPosition(handle, q)
        else:
            self.sim.setJointPosition(handle, q)

    def actuation(self):
        dt = self.sim.getSimulationTimeStep()
        # The real driver serves its topics only once the robot is initialized: messages sent before are lost.
        if self.state == 'running':
            self.spin()
            self.check_watchdog()
            if self.fault is not None:
                # deinitialize(): back to the configuration the arm had when the driver started.
                self.state = 'stopping'

        if self.state == 'initializing':
            self.first_order_step(self.initial_configuration, OPEN_LOOP_JOINT_CONTROL_GAIN, dt)
            if self.arm_error_norm(self.initial_configuration) <= INITIAL_MOVE_ERROR_NORM:
                self.state = 'running'
                self.get_logger().info('::Robot initialized.')
        elif self.state == 'stopping':
            self.first_order_step(self.q_start, OPEN_LOOP_JOINT_CONTROL_GAIN, dt)
        elif self.target is not None:
            if MODE == 'PositionControl':
                self.first_order_step(self.target, POSITION_CONTROL_GAIN, dt)
            else:  # RawPositionControl: the target, clamped to the joint limits.
                for i in range(NUMBER_OF_ARM_JOINTS):
                    self.q_ni[i] = clamp(self.target[i], self.q_min[i], self.q_max[i])
            self.gripper_step(self.target[-1], dt)

        for handle, q in zip(self.joints, self.q_ni):
            self.set_joint(handle, q)

    def sensing(self):
        # RobotDriverROS only publishes once the robot is initialized, and the topics disappear when it stops.
        if self.state != 'running':
            return
        sim = self.sim
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position = [sim.getJointPosition(h) for h in self.joints]
        msg.velocity = [sim.getJointVelocity(h) for h in self.joints]
        msg.effort = ([sim.getJointForce(h) or 0.0 for h in self.joints] if self.dynamics
                      else [0.0] * len(self.joints))
        self.joint_states_pub.publish(msg)
        self.min_pub.publish(Float64MultiArray(data=self.q_min))
        self.max_pub.publish(Float64MultiArray(data=self.q_max))


def scene_mode():
    """The mode of the scene: the value of SCENE_MODE_VARIABLE, or 'simulation' if it is not set."""
    return os.environ.get(SCENE_MODE_VARIABLE, 'simulation').strip().lower()


def sysCall_init():
    sim = require('sim')
    self.node = None
    self.ros_initialized = False
    self.fault_reported = False
    mode = scene_mode()
    if mode != ACTIVE_IN_MODE:
        if mode not in SCENE_MODES:
            print(f"Error: {SCENE_MODE_VARIABLE}='{mode}' is not one of {SCENE_MODES}. The Unitree Z1 driver is idle.")
        else:
            print(f"Unitree Z1 driver: idle ({SCENE_MODE_VARIABLE}={mode}).")
        return
    try:
        if MODE not in ('PositionControl', 'RawPositionControl'):
            raise RuntimeError(f"MODE must be 'PositionControl' or 'RawPositionControl', not '{MODE}'.")
        base = sim.getObject(ROBOT_BASE_PATH)
        joints = find_joints(sim, base, JOINT_ALIASES)

        if not rclpy.ok():
            rclpy.init()
            self.ros_initialized = True
        self.node = UnitreeZ1Driver(sim, joints)

        print(f"Unitree Z1 driver on '{TOPIC_PREFIX}' (base='{sim.getObjectAlias(base, 1)}', mode={MODE}).")
    except Exception as e:
        print(f"Error initializing the Unitree Z1 driver script: {e}")


def sysCall_actuation():
    if self.node is None:
        return
    self.node.actuation()
    if self.node.fault is not None and not self.fault_reported:
        print(f"Unitree Z1 driver stopped: {self.node.fault}")
        self.fault_reported = True
        # The topics disappear, but actuation() keeps moving the arm back in the scene.
        self.node.ros_executor.shutdown()
        self.node.destroy_node()
        self.node.destroyed = True


def sysCall_sensing():
    if self.node is None or getattr(self.node, 'destroyed', False):
        return
    self.node.sensing()


def sysCall_cleanup():
    if self.node is not None and not getattr(self.node, 'destroyed', False):
        self.node.ros_executor.shutdown()
        self.node.destroy_node()
    self.node = None
    if self.ros_initialized:
        rclpy.shutdown()
        self.ros_initialized = False
    print("ROS 2 cleaned up")
