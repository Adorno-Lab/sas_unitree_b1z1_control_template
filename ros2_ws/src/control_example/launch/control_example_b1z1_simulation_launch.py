"""
This file is based on the sas_kuka_control_template
https://github.com/MarinhoLab/sas_kuka_control_template/blob/main/launch/real_robot_launch.py

Run this script in a different terminal window or tab. Be ready to close this, as this activates the real robot if the
connection is successful.
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
                    'sigterm_timeout',
                    default_value='30'
                ),
        Node(
            package='control_example',
            executable='control_example_node',
            name='b1z1_1',
            namespace="",
            output='screen',  
            emulate_tty=True,  
            parameters=[{
                "x_prefix": "sas_robot_driver_coppeliasim/object/x1",
                "xd_prefix": "sas_robot_driver_coppeliasim/object/xd1",
                "trunk_to_arm_base_translation": [0.2964, 0.0, 0.1617],
                "B1_topic_prefix": "sas_b1/b1_1",
                "Z1_topic_prefix": "sas_z1/z1_1",
                "thread_sampling_time_sec": 0.002,
                "configuration_limits_min_deg": [-10000.0, -10000.0, -10000.0, -30.0,  45.0,  -90.0, -80.0, -45.0, -160.0],  
                "configuration_limits_max_deg": [ 10000.0,  10000.0,  10000.0,  30.0, 165.0,    0.0,  80.0,  45.0,  160.0],  
                "config_vel_limits_min": [-0.15, -0.15, -0.15, -1.57, -1.57, -1.57, -1.57, -1.57, -1.57],
                "config_vel_limits_max": [ 0.2,  0.15,  0.15,  1.57,  1.57,  1.57,  1.57,  1.57,  1.57],
            }]
        ),

    ])
