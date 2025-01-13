FROM osrf/ros:humble-desktop

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install --no-install-recommends -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-joint-state-publisher \
    ros-humble-xacro \
    python3-pip \
    ros-humble-domain-bridge \
    ros-humble-nav2-msgs \
    && rm -rf /var/lib/apt/lists/*

RUN pip install setuptools==58.2.0

COPY ./antdrone_gz/ /home/humble_ws/src/antdrone_gz/
COPY ./antdrone_description/ /home/humble_ws/src/antdrone_description
COPY ./antdrone_bringup /home/humble_ws/src/antdrone_bringup
COPY ./antdrone_localization/ /home/humble_ws/src/antdrone_localization
RUN git clone https://github.com/keenan88/IFRA_LinkAttacher /home/humble_ws/src/IFRA_LinkAttacher

WORKDIR /home/humble_ws

RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install && \
    source install/setup.bash

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /home/humble_ws/install/setup.bash" >> ~/.bashrc

CMD bash -c "source /home/humble_ws/install/setup.bash && \
    ros2 launch antdrone_bringup spawn_drone.launch.py world_name:=${world_name} DRONE_NAME:=${DRONE_NAME}"