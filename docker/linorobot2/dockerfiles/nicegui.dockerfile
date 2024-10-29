FROM osrf/ros:humble-desktop

RUN apt-get update && apt-get install -y \
    python3 \
    python3-pip \
    ros-humble-rosbridge-server

RUN pip install nicegui 

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc