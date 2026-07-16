FROM murilomarinho/sas:jazzy
SHELL ["/bin/bash", "-c"]
ENV BASH_ENV="/etc/bash_env"



########################################################################################################
# Update SAS
RUN apt update && apt-get install ros-jazzy-sas-* -y



########################################################################################################
# Create the folder /app if not present
WORKDIR /app


########################################################################################################
# Create the folder /app/software/ROS2/ros2_ws/src if not present
RUN mkdir -p /app/software/ROS2/ros2_ws/src




########################################################################################################

# Clone the sas_tools repository
RUN cd /app/software/ROS2/ros2_ws/src && git clone https://github.com/Adorno-Lab/sas_tools --recursive

# Clone the sas_robot_driver_unitree_z1
RUN cd /app/software/ROS2/ros2_ws/src && git clone https://github.com/Adorno-Lab/sas_robot_driver_unitree_z1 --recursive

# Clone the sas_robot_driver_unitree_b1
RUN cd /app/software/ROS2/ros2_ws/src && git clone https://github.com/Adorno-Lab/sas_robot_driver_unitree_b1 --recursive

# Set environment variables properly using ENV
ENV LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:/app/software/ROS2/ros2_ws/src/sas_robot_driver_unitree_z1/src/z1_sdk/lib/:/app/software/ROS2/ros2_ws/src/sas_robot_driver_unitree_b1/src/unitree_legged_sdk/lib/cpp/amd64/"
ENV LIBRARY_PATH="${LIBRARY_PATH}:/app/software/ROS2/ros2_ws/src/sas_robot_driver_unitree_z1/src/z1_sdk/lib/:/app/software/ROS2/ros2_ws/src/sas_robot_driver_unitree_b1/src/unitree_legged_sdk/lib/cpp/amd64/"
##########################################################################################



# Create the git folder to clone github repositories if not present.
#######################################################################################################
RUN mkdir -p ~/git



# Clone the rap2025_wholebodycontrol and copy the EKF package 
#######################################################################################################
RUN cd ~/git && git clone https://github.com/Adorno-Lab/rap2025_wholebodycontrol.git --recursive
RUN cp -r ~/git/rap2025_wholebodycontrol/software/ROS2/ros2_ws/src/sas_extended_kalman_filter_unitree_b1 /app/software/ROS2/ros2_ws/src



# Clone the sas_unitree_b1z1_control_template and copy the pacakges in the ros2_ws/src/ folder
#######################################################################################################
#RUN cd ~/git && \
#    git clone https://github.com/Adorno-Lab/sas_unitree_b1z1_control_template && \
#    mkdir -p /app/software/ROS2/ros2_ws/src && \
#    cp -r ~/git/sas_unitree_b1z1_control_template/ros2_ws/src/sas_unitree_b1z1_control_template /app/software/ROS2/ros2_ws/src/


# Copy the template package in the ros2_ws/src/ folder
#################################################################################################
WORKDIR /app

RUN mkdir -p /app/software/ROS2/ros2_ws/src

COPY ./ros2_ws/src/sas_unitree_b1z1_control_template /app/software/ROS2/ros2_ws/src/sas_unitree_b1z1_control_template
COPY ./utils /app/software/ROS2/utils  


##########################################################################################
# Install Unitree kinematics

RUN cd ~/git && git clone https://github.com/Adorno-Lab/unitree_kinematics.git
RUN cd ~/git/unitree_kinematics && mkdir -p build && cd build && cmake .. && make && make install



# Add some useful shortcuts
RUN echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash_env
RUN echo "# Source the ROS2 overlay, as instructed in https://github.com/SmartArmStack/smart_arm_stack_ROS2" >> /etc/bash_env
RUN echo "alias cdros2='cd /app/software/ROS2/ros2_ws/' " >> /etc/bash_env
RUN echo "alias buildros2='cdros2 && colcon build && source install/setup.bash' " >> /etc/bash_env

# Build the ROS2 packages
RUN bash -c "source /opt/ros/jazzy/setup.bash; cd /app/software/ROS2/ros2_ws/; colcon build && source install/setup.bash" 

RUN echo "source /etc/bash_env" >> ~/.bashrc
