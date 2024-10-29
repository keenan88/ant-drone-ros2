FROM ros:noetic

RUN apt-get update && apt-get install -y \
    ros-noetic-map-server \
    ros-noetic-rosbridge-server \
    git

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc