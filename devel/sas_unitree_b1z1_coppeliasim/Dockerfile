# CoppeliaSim 4.7 (rev4) image to run scenes/unitree_b1z1_ros2.ttt. The base image already provides ROS 2 Jazzy,
# SAS, DQ Robotics, and the Python packages (pyzmq, cbor2) that CoppeliaSim needs for Python child scripts.
#
# Build, e.g. from the repository root:
#   docker build -f devel/sas_unitree_b1z1_coppeliasim/Dockerfile -t sas_unitree_b1z1_coppeliasim .
FROM murilomarinho/coppeliasim:noble_470rev4
SHELL ["/bin/bash", "-c"]
ENV BASH_ENV="/etc/bash_env"
RUN echo "export ROS_DOMAIN_ID=77" >> /etc/bash_env

# Update the SAS packages of the base image. Only those already installed. The debug symbols are removed first,
# as they pin the old versions.
RUN apt-get update && \
    apt-get purge -y 'ros-jazzy-sas-*-dbgsym' && \
    apt-get install -y --only-upgrade 'ros-jazzy-sas-*' && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Messages imported by the Python child scripts of the scene (scenes/scripts) through rclpy: sas_msgs (base image)
# and sas_legged_msgs (sas_tools, built here). CoppeliaSim runs those scripts with the system python3, so it finds
# the messages in the environment of the shell that starts it.
RUN mkdir -p /root/ros2_ws/src && \
    cd /root/ros2_ws/src && \
    git clone https://github.com/Adorno-Lab/sas_tools.git && \
    cd /root/ros2_ws && \
    source /opt/ros/jazzy/setup.bash && \
    colcon build --packages-select sas_legged_msgs --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    echo "source /root/ros2_ws/install/setup.bash" >> /etc/bash_env

RUN echo "source /etc/bash_env" >> ~/.bashrc

# The scenes are mounted here by the compose file (not copied), so that the changes saved from CoppeliaSim land in
# the repository.
RUN mkdir -p /app/scenes
WORKDIR /app/scenes
