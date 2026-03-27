ARG CUDA_VERSION=12.8
FROM pytorch/pytorch:2.7.0-cuda12.8-cudnn9-devel AS builder
# FROM pytorch/pytorch:2.5.1-cuda${CUDA_VERSION}-cudnn9-devel AS builder
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        libgl1 \
        libglib2.0-0 \
        git \
        wget \
        curl \
        unzip \
        ca-certificates && \
    rm -rf /var/lib/apt/lists/*
RUN wget https://download.pytorch.org/libtorch/cu128/libtorch-cxx11-abi-shared-with-deps-2.7.0%2Bcu128.zip && \
    unzip  libtorch-cxx11-abi-shared-with-deps-2.7.0+cu128.zip && \
    rm libtorch-cxx11-abi-shared-with-deps-2.7.0+cu128.zip && \
    mv libtorch /usr/local/


# This prevents the timezone prompt from hanging the build
ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=UTC

# 1. Setup Locale for ROS 2
RUN apt-get update && apt-get install -y locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# 2. Install basic dependencies
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        libgl1 \
        libglib2.0-0 \
        git \
        wget \
        curl \
        unzip \
        ca-certificates \
        gnupg2 \
        lsb-release && \
    rm -rf /var/lib/apt/lists/*

# 3. Add ROS 2 Humble GPG key and Repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 4. Install ROS 2 Humble (Base version for robotics development)
# Also installing colcon for building C++/LibTorch nodes
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-base \
    ros-dev-tools \
    python3-colcon-common-extensions && \
    rm -rf /var/lib/apt/lists/*

RUN rosdep init && rosdep update

RUN apt-get update && apt-get install -y libopencv-dev ros-humble-sophus vim && rm -rf /var/lib/apt/lists/*

ENV PATH=/usr/local/nvidia/bin:/usr/local/cuda/bin:/usr/local/nvidia/bin:/usr/local/cuda/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin

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

WORKDIR /ros2_ws/src/gs-lio

COPY . .

# Install essential for xpra
RUN apt-get update && apt-get install -y \
    openssh-server sudo \
    libx11-6 libxcb1 libglu1-mesa

RUN apt-get install -y xvfb x11vnc && apt-get install -y xpra && apt-get install -y python3-uinput && apt-get install -y dbus-x11 && apt-get update && apt-get install -y libgl1-mesa-glx libglx-mesa0 libglu1-mesa libgl1-mesa-dri mesa-utils && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y xserver-xorg xinit xterm openbox

RUN echo 'openbox & exec xterm' > /root/.xsession && chmod +x /root/.xsession

# Install mimalloc
RUN cd /tmp && git clone https://github.com/microsoft/mimalloc.git && \
    cd /tmp/mimalloc && mkdir -p out/release && cd out/release && cmake ../.. && make && make install && cd /tmp && rm -r mimalloc

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

CMD ["/entrypoint.sh"]
