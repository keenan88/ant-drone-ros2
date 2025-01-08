FROM osrf/ros:humble-desktop

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install --no-install-recommends -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-joint-state-publisher \
    ros-humble-xacro \
    python3-pip \
    netcat \
    ros-humble-domain-bridge \
    ros-humble-nav2-msgs \
    nvidia-driver-535 \
    && rm -rf /var/lib/apt/lists/*

RUN pip install setuptools==58.2.0

# Simulation models is a big repo, clone it before build to avoid re-cloning in every build
WORKDIR /home/models/
RUN git clone https://github.com/osrf/gazebo_models.git
RUN git clone https://github.com/koide3/gazebo_apriltag.git


COPY ./antdrone_description/ /home/humble_ws/src/antdrone_description
RUN git clone https://github.com/keenan88/IFRA_LinkAttacher /home/humble_ws/src/IFRA_LinkAttacher
COPY ./simulation /home/simulation


WORKDIR /home/humble_ws

RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install && \
    source install/setup.bash

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /home/humble_ws/install/setup.bash" >> ~/.bashrc

ENV GAZEBO_MODEL_PATH=/home/humble_ws/src/:/home/simulation/models:/home/models/gazebo_models:/home/models/gazebo_apriltag/models:/home/humble_ws/build/

CMD bash -c "source /home/humble_ws/install/setup.bash && \
             gzserver --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so /home/simulation/gazebo/${world_name}.world"