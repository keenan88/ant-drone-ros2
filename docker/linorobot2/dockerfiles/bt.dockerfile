FROM osrf/ros:humble-desktop

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get install -y \
    python3-pip \
    curl \
    python3-colcon-mixin \
    ros-humble-behaviortree-cpp \
    ros-humble-rmf-dev \
    ros-humble-generate-parameter-library \
    ros-humble-domain-bridge \
    ros-humble-nav2-msgs \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install setuptools==58.2.0


COPY ./linorobot2/floor_mission_bt /home/humble_ws/src/floor_mission_bt
COPY ./linorobot2/floor_mission_helper /home/humble_ws/src/floor_mission_helper
COPY ./linorobot2/antdrone_interfaces /home/humble_ws/src/antdrone_interfaces
COPY ./ant_queen/ant_queen_interfaces /home/humble_ws/src/ant_queen_interfaces

WORKDIR /home/humble_ws/src
RUN git clone https://github.com/keenan88/IFRA_LinkAttacher
RUN git clone --branch humble https://github.com/BehaviorTree/BehaviorTree.ROS2.git

WORKDIR /home/humble_ws/

RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install && \
    source install/setup.bash

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /home/humble_ws/install/setup.bash" >> ~/.bashrc

CMD bash -c "source /home/humble_ws/install/setup.bash && \
            ros2 launch floor_mission_bt bt.launch.py DRONE_NAME:=${DRONE_NAME}"
