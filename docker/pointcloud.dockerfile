FROM ros:humble-ros-base

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

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

COPY ./antdrone_pcl /home/humble_ws/src/antdrone_pcl
COPY ./perception_pcl /home/humble_ws/src/perception_pcl

WORKDIR /home/humble_ws

RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install && \
    source install/setup.bash && \
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /home/humble_ws/install/setup.bash" >> ~/.bashrc

CMD bash -c "source /home/humble_ws/install/setup.bash && \
            ros2 launch antdrone_pcl pointcloud.launch.py USE_SIM_TIME:=${USE_SIM_TIME}"