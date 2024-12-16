FROM ros:humble-ros-base

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

# Do installs before git clone to avoid needing to do a full image rebuild each time image is built
RUN apt-get update && apt-get install --no-install-recommends -y \
    nano \
    ros-humble-image-pipeline \
    ros-humble-domain-bridge \
    python3-pip \
    libpcl-dev \
    libpcl-common1.12 \
    libpcl-io1.12 \
    libpcl-features1.12 \
    libpcl-filters1.12 \
    libpcl-segmentation1.12 \
    libpcl-surface1.12 \
    ros-humble-pcl-msgs \
    nano \  
    && rm -rf /var/lib/apt/lists/*

RUN pip install setuptools==58.2.0

WORKDIR /home/humble_ws

COPY ./linorobot2_pcl /home/humble_ws/src/linorobot2_pcl
COPY ./perception_pcl /home/humble_ws/src/perception_pcl

# Build the workspace and source the setup files
RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install && \
    source install/setup.bash && \
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /home/humble_ws/install/setup.bash" >> ~/.bashrc

CMD bash -c "source /home/humble_ws/install/setup.bash && \
            ros2 launch linorobot2_pcl pointcloud.launch.py USE_SIM_TIME:=${USE_SIM_TIME}"