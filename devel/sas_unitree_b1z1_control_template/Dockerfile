FROM juanjqo/sas_unitree_b1z1_jazzy
SHELL ["/bin/bash", "-c"]
ENV BASH_ENV="/etc/bash_env"

RUN echo "export ROS_DOMAIN_ID=77" >> /etc/bash_env



# Update SAS
RUN apt update && apt-get install ros-jazzy-sas-* -y


# Install QT
RUN apt update && apt-get install qt6-base-dev qt6-tools-dev-tools -y


# Install robot_constraint_editor
RUN cd ~/git && git clone https://github.com/Adorno-Lab/robot_constraint_editor
RUN cd ~/git/robot_constraint_editor && mkdir -p build && cd build && cmake .. && make && make install



# Clone the robot_constraint_manager package
RUN cd ~/git && git clone https://github.com/Adorno-Lab/robot_constraint_manager 
RUN cd ~/git/robot_constraint_manager && mkdir -p build && cd build && cmake .. && make && make install



WORKDIR /app

RUN mkdir -p /app/software/ROS2/ros2_ws/src

COPY ./ros2_ws/src/ /app/software/ROS2/ros2_ws/src/
COPY ./utils /app/software/ROS2/utils  

RUN bash -c "source /opt/ros/jazzy/setup.bash; cd /app/software/ROS2/ros2_ws/; colcon build && source install/setup.bash" 


RUN ldconfig
RUN echo "source /etc/bash_env" >> ~/.bashrc





