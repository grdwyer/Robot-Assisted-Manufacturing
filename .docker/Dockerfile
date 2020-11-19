ARG ros_distro=foxy

FROM ros:foxy

ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV DEBIAN_FRONTEND=noninteractive


# Install any needed packages then clean up apt cache
RUN apt-get update && apt-get install -y \
    software-properties-common \
    ros-foxy-rqt \
    ros-foxy-vision-opencv \
    ssh \
    git \
    python3-pip \
    libboost-all-dev \
    python3-vcstool \
    ros-foxy-perception-pcl \
    && rm -rf /var/lib/apt/lists/*
 
# Add PPA for QTScxml library for ros_scxml
# RUN apt-add-repository ppa:beineri/opt-qt-5.14.2-focal && \
#     apt-get update && apt-get install -y \
#     qt514-meta-full \
#     && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    libyaml-cpp-dev \
    ros-foxy-octomap \
    libassimp-dev \
    libeigen3-dev \
    coinor-libipopt-dev \
    bison \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# Copies perfusion system stack into image
RUN mkdir -p /dev_ws/src/ram
COPY . /dev_ws/src/ram/

# Build the workspace so you're ready to go
WORKDIR /dev_ws
RUN ["/bin/bash", "-c", "source /opt/ros/foxy/setup.bash &&\
    vcs import src < ./src/ram/ram.rosinstall &&\
    vcs pull src &&\
    colcon build"]

CMD ["bash"]