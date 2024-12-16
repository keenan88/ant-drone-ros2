FROM osrf/ros:humble-desktop

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get install -y \
    nano \
    # ros-humble-realsense2-camera \ 
    # ros-humble-realsense2-description \
    # usbutils \
    python3-pip -y \
    ros-humble-pointcloud-to-laserscan \
    # ros-humble-depthimage-to-laserscan \
    # ros-humble-joint-state-publisher \
    # ros-humble-controller-manager \
    # ros-humble-joint-state-publisher-gui \
    && rm -rf /var/lib/apt/lists/*

RUN pip install setuptools==58.2.0

WORKDIR /home/humble_ws

COPY ./antdrone_depth_to_laserscan /home/humble_ws/src/antdrone_depth_to_laserscan
# COPY ./laser_filters /home/humble_ws/src/laser_filters

# Build the workspace and source the setup files
RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install --packages-select antdrone_depth_to_laserscan && \
    source install/setup.bash && \
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /home/humble_ws/install/setup.bash" >> ~/.bashrc

CMD bash -c "source /home/humble_ws/install/setup.bash && \
            ros2 launch antdrone_depth_to_laserscan depth_to_laserscan.launch.py USE_SIM_TIME:=${USE_SIM_TIME}"
