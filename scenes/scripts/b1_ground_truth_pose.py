#python
"""
CoppeliaSim 4.7 Python child script: publishes the pose of the trunk of the B1 (ground truth) on the topic
of the pose estimate used in the lab (sas_extended_kalman_filter_unitree_b1, with the Vicon system):
    TOPIC (geometry_msgs/PoseStamped, frame 'world')

It stands in for the localization of the real robot, which is not part of the B1 driver. Do not run it
together with the real localization on the same ROS_DOMAIN_ID.

Attach it as a non-threaded Python child script to the trunk of the B1 (trunk_respondable), or set
OBJECT_PATH. Launch CoppeliaSim from a terminal where ROS 2 is sourced.

Scene mode: this script only runs when the environment variable B1Z1_SCENE_MODE is 'simulation' (or not
set) when CoppeliaSim starts. In 'mirror', it is idle, and sas_unitree_b1z1_mirror.py shows the real robot.
"""

import os

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

SCENE_MODE_VARIABLE = 'B1Z1_SCENE_MODE'
SCENE_MODES = ('simulation', 'mirror')
ACTIVE_IN_MODE = 'simulation'

TOPIC = '/sas_b1/b1_1/get/ekf/robot_pose'
NODE_NAME = 'coppeliasim_b1_1_ground_truth_pose'
FRAME_ID = 'world'

# '..' = the parent of this script object, i.e. the object the script is attached to.
OBJECT_PATH = '..'


class GroundTruthPose(Node):

    def __init__(self, sim, object_handle):
        super().__init__(NODE_NAME)
        self.sim = sim
        self.object_handle = object_handle
        self.publisher = self.create_publisher(PoseStamped, TOPIC, 1)
        self.ros_executor = SingleThreadedExecutor()
        self.ros_executor.add_node(self)

    def sensing(self):
        x, y, z, qx, qy, qz, qw = self.sim.getObjectPose(self.object_handle, self.sim.handle_world)
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = FRAME_ID
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = x, y, z
        msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z = qw, qx, qy, qz
        self.publisher.publish(msg)


def scene_mode():
    """The mode of the scene: the value of SCENE_MODE_VARIABLE, or 'simulation' if it is not set."""
    return os.environ.get(SCENE_MODE_VARIABLE, 'simulation').strip().lower()


def sysCall_init():
    sim = require('sim')
    self.node = None
    self.ros_initialized = False
    mode = scene_mode()
    if mode != ACTIVE_IN_MODE:
        if mode not in SCENE_MODES:
            print(f"Error: {SCENE_MODE_VARIABLE}='{mode}' is not one of {SCENE_MODES}. The ground truth pose is idle.")
        return
    try:
        object_handle = sim.getObject(OBJECT_PATH)
        if not rclpy.ok():
            rclpy.init()
            self.ros_initialized = True
        self.node = GroundTruthPose(sim, object_handle)
        print(f"Ground truth pose of '{sim.getObjectAlias(object_handle, 1)}' on '{TOPIC}'.")
    except Exception as e:
        print(f"Error initializing the ground truth pose script: {e}")


def sysCall_sensing():
    if self.node is None:
        return
    self.node.sensing()


def sysCall_cleanup():
    if self.node is not None:
        self.node.ros_executor.shutdown()
        self.node.destroy_node()
        self.node = None
    if self.ros_initialized:
        rclpy.shutdown()
        self.ros_initialized = False
