# FROM osrf/ros:humble-desktop AS base

# on arm platform
FROM ros:humble AS base

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

# Install colcon + rosdep + git
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    git \
    && rm -rf /var/lib/apt/lists/*

# Init rosdep (safe for Docker)
RUN rosdep update

RUN apt-get update && apt-get install -y ros-$ROS_DISTRO-sophus
# Create workspace
WORKDIR /ros2_ws
RUN mkdir -p src/gs-lio

RUN apt-get update && apt-get install -y \
    libeigen3-dev

RUN cd / && git clone https://github.com/Livox-SDK/Livox-SDK2.git && \
    cd ./Livox-SDK2/ && mkdir build && cd build && \
    cmake .. && make -j && make install && cd / && rm -r Livox-SDK2


RUN cd src && apt-get update && apt-get install -y ros-humble-pcl-ros && \
    git clone https://github.com/Livox-SDK/livox_ros_driver2.git && \
    . /opt/ros/humble/setup.sh && cd livox_ros_driver2 && ./build.sh humble

RUN apt-get update && apt-get install -y ros-humble-compressed-image-transport
# Build

WORKDIR /ros2_ws/src/gs-lio

COPY . .

RUN apt-get update && apt-get install -y \
    openssh-server sudo \
    libx11-6 libxcb1 libglu1-mesa

RUN apt-get install -y xvfb x11vnc && apt-get install -y xpra && apt-get install -y python3-uinput && apt-get install -y dbus-x11 && apt-get update && apt-get install -y libgl1-mesa-glx libglx-mesa0 libglu1-mesa libgl1-mesa-dri mesa-utils && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y xserver-xorg xinit xterm openbox

RUN echo 'openbox & exec xterm' > /root/.xsession && chmod +x /root/.xsession

RUN cd /tmp && git clone https://github.com/microsoft/mimalloc.git && \
    cd /tmp/mimalloc && mkdir -p out/release && cd out/release && cmake ../.. && make && make install && cd /tmp && rm -r mimalloc


COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

CMD ["/entrypoint.sh"]
