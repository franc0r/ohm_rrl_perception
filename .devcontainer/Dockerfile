FROM ros:noetic-perception

# Environment variables
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# Install tzdate
RUN apt-get update \
    && apt-get -y install \
    qt5-default \
    ros-noetic-find-object-2d \
    git \
    && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc

RUN mkdir -p /root/catkin_ws