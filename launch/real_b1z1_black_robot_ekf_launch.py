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
    real_z1_black_robot_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('sas_robot_driver_unitree_z1'), 'launch'),
                '/real_z1_black_robot_launch.py'])
        )
    real_b1_black_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('sas_robot_driver_unitree_b1'), 'launch'),
            '/real_b1_black_robot_launch.py'])
    )
    unitree_b1_black_vicon_ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('sas_extended_kalman_filter_unitree_b1'), 'launch'),
            '/unitree_b1_black_vicon_ekf_launch.py'])
    )       
    return LaunchDescription([
        DeclareLaunchArgument(
            'sigterm_timeout',
            default_value='40'
        ),
        real_z1_black_robot_launch,
        real_b1_black_robot_launch,
        unitree_b1_black_vicon_ekf_launch
    ])
