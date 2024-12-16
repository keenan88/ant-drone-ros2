FROM osrf/ros:humble-desktop

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get install -y \
    python3-pip \
    curl \
    python3-colcon-mixin \
    ros-dev-tools \
    ros-humble-rmf-dev \
    ros-humble-gazebo-plugins \
    python3-paho-mqtt \
    python3-flask-socketio \
    python3-flask \
    python3-flask-cors \
    python3-websockets \
    ros-humble-ros-ign-bridge \ 
    ros-humble-nav2-bringup \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install flask-socketio fastapi uvicorn setuptools==58.2.0

WORKDIR /home/humble_ws/

COPY ./linorobot2/antdrone_queen_client /home/humble_ws/src/antdrone_queen_client
COPY ./linorobot2/antdrone_interfaces /home/humble_ws/src/antdrone_interfaces
COPY ./ant_queen/ant_queen_interfaces /home/humble_ws/src/ant_queen_interfaces
RUN git clone https://github.com/keenan88/IFRA_LinkAttacher

RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install && \
    source install/setup.bash && \
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /home/humble_ws/install/setup.bash" >> ~/.bashrc

CMD bash -c "source /home/humble_ws/install/setup.bash && \
            ros2 launch antdrone_queen_client antdrone_queen_client.launch.py DRONE_NAME:=${DRONE_NAME} USE_SIM_TIME:=${USE_SIM_TIME} FLEET_NAME:=${FLEET_NAME}"

