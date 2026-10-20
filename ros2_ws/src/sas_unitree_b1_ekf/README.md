# sas_unitree_b1_ekf

Basic planar EKF (x, y, phi) for the Unitree B1. Based on `sas_extended_kalman_filter_unitree_b1` (rap2025_wholebodycontrol).

- Prediction: measured twist of the B1 driver (`<prefix>/get/twist`, read with `LeggedRobotDriverClient`).
- Correction: rear and front Vicon markers (tf from `motion_capture_tracking`). If one marker is lost, the other one is used. If both are lost, only the prediction is used.
- Output: pose of the trunk on `<prefix>/get/ekf/robot_pose` (`geometry_msgs/PoseStamped`, frame `world`). The height comes from Vicon.

At the start, the robot must be still and both markers visible.

```shell
ros2 launch sas_unitree_b1_ekf unitree_b1_white_ekf_launch.py
ros2 launch sas_unitree_b1_ekf unitree_b1_black_ekf_launch.py
```

| Parameter | Description |
|---|---|
| `b1_topic_prefix` | e.g. `/sas_b1/b1_1` |
| `thread_sampling_time_sec` | Sampling time of the loop |
| `world_frame` | tf frame of Vicon |
| `robot_vicon_marker_rear`, `robot_vicon_marker_front` | tf frames of the markers |
| `marker_timeout_sec` | A marker without new data for this long is lost |
| `max_marker_tilt_rad` | More tilted marker samples are rejected |
| `initialization_samples` | Vicon samples averaged at the start |
| `rear_marker_to_trunk_translation` | Translation from the rear marker to the trunk, in the frame of the marker |
| `process_noise`, `measurement_noise` | Diagonals of R (per iteration) and Q |
