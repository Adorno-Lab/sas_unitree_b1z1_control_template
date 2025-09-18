FROM murilomarinho/sas:jazzy
SHELL ["/bin/bash", "-c"]
ENV BASH_ENV="/etc/bash_env"




# Clone sas_unitree_z1_control_template
RUN cd ~/ros2_ws/src && git clone https://github.com/Adorno-Lab/sas_unitree_b1z1_control_template.git --recursive

# Clone the rap2025_wholebodycontrol.
RUN mkdir -p ~/git
RUN cd ~/git && git clone https://github.com/Adorno-Lab/rap2025_wholebodycontrol.git --recursive
RUN cp -r ~/git/rap2025_wholebodycontrol/software/ROS2/ros2_ws/src/sas_extended_kalman_filter_unitree_b1 ~/ros2_ws/src
RUN cp -r ~/git/rap2025_wholebodycontrol/software/ROS2/ros2_ws/src/sas_control_unitree_b1z1 ~/ros2_ws/src


RUN chmod +x ~/git/rap2025_wholebodycontrol/dependencies/install_sas_drivers_b1z1.sh
RUN bash -c '~/git/rap2025_wholebodycontrol/dependencies/install_sas_drivers_b1z1.sh' 

RUN chmod +x ~/git/rap2025_wholebodycontrol/dependencies/install_robot_constraint_manager.sh
RUN bash -c '~/git/rap2025_wholebodycontrol/dependencies/install_robot_constraint_manager.sh' 



RUN bash -c "echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash_env \
             echo "# Source the ROS2 overlay, as instructed in https://github.com/SmartArmStack/smart_arm_stack_ROS2" >> /etc/bash_env \
             echo "alias cdros2='cd ~/ros2_ws/' " >> /etc/bash_env \
             echo "alias buildros2='cdros2 && colcon build && source install/setup.bash' " >> /etc/bash_env "

RUN echo "source /etc/bash_env" >> ~/.bashrc

# Build the ROS2 packages
RUN bash -c "source /opt/ros/jazzy/setup.bash; cd ~/ros2_ws/; colcon build && source install/setup.bash" 
RUN echo "alias launch_echo_ROS2_drivers='cdros2 && source install/setup.bash && ros2 launch sas_unitree_b1z1_control_template real_b1z1_robot_ekf_echo_launch.py ' " >> /etc/bash_env
RUN echo "alias launch_ROS2_drivers='cdros2 && source install/setup.bash && ros2 launch sas_unitree_b1z1_control_template real_b1z1_robot_ekf_launch.py ' " >> /etc/bash_env
RUN echo "alias launch_kinematic_control='cdros2 && source install/setup.bash && ros2 launch sas_unitree_b1z1_control_template real_b1z1_robot_ekf_control_launch.py ' " >> /etc/bash_env
RUN echo "export ROS_DOMAIN_ID=77" >> /etc/bash_env
RUN echo "source /etc/bash_env" >> ~/.bashrc


