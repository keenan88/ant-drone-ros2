FROM osrf/ros:humble-desktop

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get install -y \
    python3-pip \
    curl \
    ros-humble-usb-cam \
    ros-humble-camera-calibration \
    ros-humble-ament-cmake-clang-format \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install numpy scipy setuptools==58.2.0

COPY ./antdrone_bringup /home/humble_ws/src/antdrone_bringup

WORKDIR /home/humble_ws/

RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install && \
    source install/setup.bash

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /home/humble_ws/install/setup.bash" >> ~/.bashrc

CMD bash -c "source /home/humble_ws/install/setup.bash && \
    ros2 launch antdrone_bringup usbcam.launch.py"
