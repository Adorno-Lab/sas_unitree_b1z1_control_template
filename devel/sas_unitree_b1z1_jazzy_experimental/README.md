# Specs


This image contains the SAS drivers for the B1 and Z1 platforms and is based on Ubuntu 24.04 AMD64, ROS2 Jazzy, and DQ Robotics. The image is powered by the following repositories.


- https://github.com/Adorno-Lab/rap2025_wholebodycontrol
- https://github.com/Adorno-Lab/sas_unitree_b1z1_control_template
- https://github.com/Adorno-Lab/sas_robot_driver_unitree_b1
- https://github.com/Adorno-Lab/sas_robot_driver_unitree_z1


Some features: 

- The ROS2 workspace folder is located in /app/software/ROS2/ros2_ws/
- The ROS_DOMAIND_ID is not defined. You are required to set it in your image. For instance:

```docker

FROM juanjqo/sas_unitree_b1z1_jazzy
SHELL ["/bin/bash", "-c"]
ENV BASH_ENV="/etc/bash_env"

RUN echo "export ROS_DOMAIN_ID=77" >> /etc/bash_env

### Your additional components

RUN echo "source /etc/bash_env" >> ~/.bashrc
```


