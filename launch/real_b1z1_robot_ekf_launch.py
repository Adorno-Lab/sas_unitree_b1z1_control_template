"""
This file is based on the sas_kuka_control_template
https://github.com/MarinhoLab/sas_kuka_control_template/blob/main/launch/dummy_move_in_coppeliasim_example_launch.py

Run this script in a different terminal window or tab. Be ready to close this, as this activates the real robot if the
connection is successful.
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    real_z1_robot_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('sas_robot_driver_unitree_z1'), 'launch'),
                '/real_z1_robot_launch.py'])
        )
    real_b1_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('sas_robot_driver_unitree_b1'), 'launch'),
            '/real_b1_robot_launch.py'])
    )    
    return LaunchDescription([
        DeclareLaunchArgument(
            'sigterm_timeout',
            default_value='40'
        ),
        real_z1_robot_launch,
        real_b1_robot_launch,
        
        Node(
            package='sas_extended_kalman_filter_unitree_b1',
            executable='sas_extended_kalman_filter_unitree_b1_node',
            name='ekf_b1_1',
            namespace="sas_b1",
            output="screen",
            parameters=[{
                "topic_prefix": "/sas_b1/b1_1",
                "thread_sampling_time_sec": 0.001,
                "robot_vicon_marker": "B1Z1_Frame_1"
            }]
        ),
    ])
