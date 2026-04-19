ARG CUDA_VERSION=12.8
FROM pytorch/pytorch:2.7.0-cuda12.8-cudnn9-devel AS builder
# FROM pytorch/pytorch:2.5.1-cuda${CUDA_VERSION}-cudnn9-devel AS builder

# prevents the timezone prompt from hanging the build
ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=UTC

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
        libopencv-dev \
        lsb-release \
        vim \
        libeigen3-dev \
        locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    rm -rf /var/lib/apt/lists/*

ENV LANG=en_US.UTF-8

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-base \
    ros-dev-tools \
    ros-humble-sophus \
    ros-humble-pcl-ros \
    ros-humble-compressed-image-transport \
    ros-humble-foxglove-bridge \
    python3-colcon-common-extensions && \
    rm -rf /var/lib/apt/lists/*

RUN rosdep init && rosdep update

RUN wget https://download.pytorch.org/libtorch/cu128/libtorch-cxx11-abi-shared-with-deps-2.7.0%2Bcu128.zip && \
    unzip  libtorch-cxx11-abi-shared-with-deps-2.7.0+cu128.zip && \
    rm libtorch-cxx11-abi-shared-with-deps-2.7.0+cu128.zip && \
    mv libtorch /usr/local/

ENV PATH=/usr/local/nvidia/bin:/usr/local/cuda/bin:/usr/local/nvidia/bin:/usr/local/cuda/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin

# Install essentail thirdparty libraries
RUN cd /tmp && git clone https://github.com/Livox-SDK/Livox-SDK2.git && \
    cd /tmp/Livox-SDK2/ && mkdir build && cd build && \
    cmake .. && make -j && make install

RUN cd /tmp && git clone https://github.com/microsoft/mimalloc.git && \
    cd /tmp/mimalloc && mkdir -p out/release && cd out/release && cmake ../.. && make && make install

# Create workspace
WORKDIR /ros2_ws
RUN mkdir -p src/gs-lio

RUN cd src && \
    git clone https://github.com/Livox-SDK/livox_ros_driver2.git && \
    . /opt/ros/humble/setup.sh && cd livox_ros_driver2 && ./build.sh humble

WORKDIR /ros2_ws/src/gs-lio

RUN echo 'export LD_LIBRARY_PATH=/usr/local/libtorch/lib:$LD_LIBRARY_PATH' >> ~/.bashrc && \
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

CMD ["/entrypoint.sh"]