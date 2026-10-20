FROM murilomarinho/sas:jazzy
SHELL ["/bin/bash", "-c"]
ENV BASH_ENV="/etc/bash_env"

RUN echo "export ROS_DOMAIN_ID=77" >> /etc/bash_env

# The base image already has ROS 2 Jazzy, the SAS packages, DQ Robotics (with the CoppeliaSim ZMQ interface),
# marinholab::sas::core, Eigen, and yaml-cpp.

# Update the SAS packages of the base image. Only those already installed. The debug symbols are removed first,
# as they pin the old versions.
RUN apt-get update && \
    apt-get purge -y 'ros-jazzy-sas-*-dbgsym' && \
    apt-get install -y --only-upgrade 'ros-jazzy-sas-*' && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Install Qt (sas_legged_client_window of sas_tools)
RUN apt-get update && \
    apt-get install -y qt6-base-dev && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN mkdir -p ~/git

# Install robot_constraint_editor, robot_constraint_manager (it links robot_constraint_editor), and
# unitree_kinematics (dependencies of control_example)
RUN cd ~/git/ && \
    git clone https://github.com/Adorno-Lab/robot_constraint_editor.git && \
    cd ~/git/robot_constraint_editor/ && \
    cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local && \
    cmake --build build -j$(nproc) && \
    cmake --install build

RUN cd ~/git/ && \
    git clone https://github.com/Adorno-Lab/robot_constraint_manager.git && \
    cd ~/git/robot_constraint_manager/ && \
    cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local && \
    cmake --build build -j$(nproc) && \
    cmake --install build

RUN cd ~/git/ && \
    git clone https://github.com/Adorno-Lab/unitree_kinematics.git && \
    cd ~/git/unitree_kinematics/ && \
    cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local && \
    cmake --build build -j$(nproc) && \
    cmake --install build && \
    ldconfig

WORKDIR /app

RUN mkdir -p /app/software/ROS2/ros2_ws/src

# Clone sas_tools and the drivers of the B1 and the Z1 (branch new_generation)
RUN cd /app/software/ROS2/ros2_ws/src && \
    git clone https://github.com/Adorno-Lab/sas_tools.git && \
    git clone -b new_generation https://github.com/Adorno-Lab/sas_robot_driver_unitree_b1.git --recursive && \
    git clone -b new_generation https://github.com/Adorno-Lab/sas_robot_driver_unitree_z1.git --recursive

# Copy the ROS2 packages of this repository
COPY ./ros2_ws/src/ /app/software/ROS2/ros2_ws/src/

RUN source /opt/ros/jazzy/setup.bash && \
    cd /app/software/ROS2/ros2_ws/ && \
    colcon build

# Add some useful shortcuts
RUN echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash_env
RUN echo "source /app/software/ROS2/ros2_ws/install/setup.bash" >> /etc/bash_env
RUN echo "alias cdros2='cd /app/software/ROS2/ros2_ws/' " >> /etc/bash_env
RUN echo "alias buildros2='cdros2 && colcon build && source install/setup.bash' " >> /etc/bash_env

RUN echo "source /etc/bash_env" >> ~/.bashrc
