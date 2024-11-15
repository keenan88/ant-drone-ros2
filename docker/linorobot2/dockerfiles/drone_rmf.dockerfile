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

COPY ./linorobot2_rmf_client /home/humble_ws/src/linorobot2_rmf_client


RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install && \
    source install/setup.bash && \
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /home/humble_ws/install/setup.bash" >> ~/.bashrc

    # ${DRONE_NAME}

CMD bash -c "source /home/humble_ws/install/setup.bash && \
            ros2 launch linorobot2_rmf_client linorobot2_rmf_client.launch.py ns:=${DRONE_NAME}"

