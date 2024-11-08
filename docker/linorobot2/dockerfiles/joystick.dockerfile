FROM osrf/ros:humble-desktop

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y \
    nano \
    ros-humble-teleop-twist-joy \
    ros-humble-domain-bridge \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN pip install setuptools==58.2.0

WORKDIR /home/humble_ws

COPY ./linorobot2_joystick /home/humble_ws/src/linorobot2_joystick

# Build the workspace and source the setup files
RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install && \
    source install/setup.bash && \
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /home/humble_ws/install/setup.bash" >> ~/.bashrc

CMD bash -c "source /home/humble_ws/install/setup.bash && \
            ros2 launch linorobot2_joystick joystick.launch.py \
            WHEELED_MOTION:=${WHEELED_MOTION}"