FROM osrf/ros:humble-desktop

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install --no-install-recommends -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-joint-state-publisher \
    ros-humble-xacro \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN pip install setuptools==58.2.0

WORKDIR /home/humble_ws

COPY ./antdrone_gz/ /home/humble_ws/src/antdrone_gz/
COPY ./antdrone_description/ /home/humble_ws/src/antdrone_description

RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install --packages-select antdrone_gz antdrone_description && \
    source install/setup.bash && \
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /home/humble_ws/install/setup.bash" >> ~/.bashrc

ENV GAZEBO_MODEL_PATH=/home/humble_ws/src/:/home/simulation/models

CMD bash -c "source /home/humble_ws/install/setup.bash && \
            ros2 launch antdrone_gz drone_greenhouse.launch.py"