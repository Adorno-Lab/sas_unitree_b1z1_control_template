"""
EKF of the B1Z1-black (sas_b1/b1_2). Requires the B1 driver and the Vicon markers on tf (motion_capture_tracking).
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sas_unitree_b1_ekf',
            executable='sas_unitree_b1_ekf_node',
            name='ekf_b1_2',
            namespace='sas_b1',
            output='screen',
            parameters=[{
                'b1_topic_prefix': '/sas_b1/b1_2',
                'thread_sampling_time_sec': 0.001,
                'world_frame': 'world',
                'robot_vicon_marker_rear': 'B1Z1_Frame_robot_black_rear',
                'robot_vicon_marker_front': 'B1Z1_Frame_robot_black_front',
                'marker_timeout_sec': 0.1,
                'max_marker_tilt_rad': 0.45,
                'initialization_samples': 100,
                'rear_marker_to_trunk_translation': [0.25, 0.0, -0.17],
                'process_noise': [0.05, 0.05, 0.05],
                'measurement_noise': [1e-3, 1e-3, 1e-3],
            }]
        )
    ])
