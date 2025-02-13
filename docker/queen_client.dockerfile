FROM osrf/ros:humble-desktop

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get install -y \
    python3-pip \
    curl \
    ros-humble-nav2-bringup \
    ros-humble-rmf-dev \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install flask-socketio fastapi uvicorn setuptools==58.2.0

COPY ./ant-drone-ros2/antdrone_queen_client /home/humble_ws/src/antdrone_queen_client
COPY ./ant-drone-ros2/antdrone_interfaces /home/humble_ws/src/antdrone_interfaces
COPY ./ant-drone-ros2/antdrone_bringup /home/humble_ws/src/antdrone_bringup
COPY ./ant-queen-ros2/ant_queen_interfaces /home/humble_ws/src/ant_queen_interfaces

WORKDIR /home/humble_ws/

RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install && \
    source install/setup.bash && \
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /home/humble_ws/install/setup.bash" >> ~/.bashrc

CMD bash -c "source /home/humble_ws/install/setup.bash && \
    ros2 launch antdrone_bringup queen_client.launch.py DRONE_NAME:=${DRONE_NAME} USE_SIM_TIME:=${USE_SIM_TIME} FLEET_NAME:=${FLEET_NAME}"

