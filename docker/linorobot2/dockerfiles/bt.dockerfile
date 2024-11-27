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
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install setuptools==58.2.0


COPY ./linorobot2/floor_mission_bt /home/humble_ws/src/floor_mission_bt
COPY ./ant_queen/ant_fleet_interfaces /home/humble_ws/src/ant_fleet_interfaces

# WORKDIR /home/humble_ws/src
# RUN git clone --branch v3.11.3 https://github.com/nlohmann/json.git

WORKDIR /home/humble_ws/src
RUN git clone https://github.com/keenan88/IFRA_LinkAttacher
RUN git clone https://github.com/BehaviorTree/BehaviorTree.ROS2.git

WORKDIR /home/humble_ws/

RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install && \
    source install/setup.bash

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /home/humble_ws/install/setup.bash" >> ~/.bashrc

CMD bash -c "source /home/humble_ws/install/setup.bash && \
            ros2 run floor_mission_bt floor_mission_bt"
