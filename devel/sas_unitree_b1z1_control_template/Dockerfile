FROM murilomarinho/sas:jazzy
SHELL ["/bin/bash", "-c"]
ENV BASH_ENV="/etc/bash_env"



RUN mkdir -p ~/git
# Clone sas_unitree_z1_control_template
RUN cd ~/git && git clone https://github.com/Adorno-Lab/sas_unitree_z1_control_template.git --recursive

# Clone the RAP 
RUN cd ~/git && git clone https://github.com/Adorno-Lab/rap2025_wholebodycontrol.git -b new_onboard_strategy --recursive
RUN cp ~/git/rap2025_wholebodycontrol/software/ROS2/ros2_ws/src/sas_extended_kalman_filter_unitree_b1 ~/ros2_ws/src
  


# Install the Z1 driver
RUN cd ~/ros2_ws/src && git clone https://github.com/Adorno-Lab/sas_robot_driver_unitree_z1 --recursive

# Install the B1 driver
RUN cd ~/ros2_ws/src && git clone https://github.com/Adorno-Lab/sas_robot_driver_unitree_b1 --recursive


#
RUN bash -c "echo "# Update the environment variable LD_LIBRARY_PATH as instructed in https://ros2-tutorial.readthedocs.io" >> /etc/bash_env; \
             echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/ros2_ws/src/sas_robot_driver_unitree_z1/src/z1_sdk/lib/:~/ros2_ws/src/sas_robot_driver_unitree_b1/src/unitree_legged_sdk/lib/cpp/amd64/" >> /etc/bash_env; \
             echo "# Update the environment variable LIBRARY_PATH as instructed in https://ros2-tutorial.readthedocs.io" >> /etc/bash_env \
             echo "export LIBRARY_PATH=$LIBRARY_PATH:~/ros2_ws/src/sas_robot_driver_unitree_z1/src/z1_sdk/lib/:~/ros2_ws/src/sas_robot_driver_unitree_b1/src/unitree_legged_sdk/lib/cpp/amd64/" >> /etc/bash_env "

RUN echo "alias launch_ROS2_B1Z1_drivers='cdros2 && source install/setup.bash && ros2 launch sas_robot_driver_unitree_b1 real_b1z1_robot_ekf_launch.py ' " >> /etc/bash_env 

RUN bash -c "echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash_env \
             echo "# Source the ROS2 overlay, as instructed in https://github.com/SmartArmStack/smart_arm_stack_ROS2" >> /etc/bash_env \
             echo "alias cdros2='cd ~/ros2_ws/' " >> /etc/bash_env \
             echo "alias buildros2='cdros2 && colcon build && source install/setup.bash' " >> /etc/bash_env "



# Build the ROS2 packages
RUN bash -c "source /opt/ros/jazzy/setup.bash; cd ~/ros2_ws/; colcon build && source install/setup.bash" 

