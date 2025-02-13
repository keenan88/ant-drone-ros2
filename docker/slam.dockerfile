FROM osrf/ros:humble-desktop

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install --no-install-recommends -y \
    git \
    python3-colcon-common-extensions \
    python3-pip \
    ros-humble-nav2-bringup \
    && rm -rf /var/lib/apt/lists/*

RUN pip install setuptools==58.2.0

WORKDIR /home/humble_ws

COPY ./antdrone_slam /home/humble_ws/src/antdrone_slam
COPY ./antdrone_localization /home/humble_ws/src/antdrone_localization
COPY ./antdrone_bringup /home/humble_ws/src/antdrone_bringup


RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install && \
    source install/setup.bash && \
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /home/humble_ws/install/setup.bash" >> ~/.bashrc

CMD bash -c "source /home/humble_ws/install/setup.bash && \
            ros2 launch antdrone_bringup slam.launch.py DRONE_NAME:=${DRONE_NAME} USE_SIM_TIME:=${USE_SIM_TIME}"