FROM osrf/ros:humble-desktop

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get install -y \
    nano \
    ros-humble-realsense2-camera \ 
    ros-humble-realsense2-description \
    usbutils \
    python3-pip -y \
    ros-humble-domain-bridge \
    && rm -rf /var/lib/apt/lists/*

RUN pip install setuptools==58.2.0

RUN mkdir -p /home/humble_ws/src
WORKDIR /home/humble_ws

COPY ./antdrone_bringup /home/humble_ws/src/antdrone_bringup
COPY ./antdrone_realsense /home/humble_ws/src/antdrone_realsense

RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install && \
    source install/setup.bash

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /home/humble_ws/install/setup.bash" >> ~/.bashrc

CMD bash -c "source /home/humble_ws/install/setup.bash && \
    ros2 launch antdrone_bringup realsense.launch.py"

