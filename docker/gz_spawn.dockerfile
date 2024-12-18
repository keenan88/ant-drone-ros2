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
RUN git clone https://github.com/keenan88/IFRA_LinkAttacher /home/humble_ws/src/IFRA_LinkAttacher

WORKDIR /home/humble_ws

RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install && \
    source install/setup.bash

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /home/humble_ws/install/setup.bash" >> ~/.bashrc

CMD bash -c "source /home/humble_ws/install/setup.bash && \
    ros2 launch antdrone_gz spawn_drone.launch.py x0:=${x0} y0:=${y0} z0:=${z0} yaw0:=${yaw0} DRONE_NAME:=${DRONE_NAME}"