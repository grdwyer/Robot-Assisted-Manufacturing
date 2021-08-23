ARG ROSDISTRO=foxy

FROM ros:$ROSDISTRO

ARG ROSDISTRO
ARG MOVEIT2=true
ARG TESSERACT=false

ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV DEBIAN_FRONTEND=noninteractive


# Install any needed packages then clean up apt cache
# Build tools and general ROS tools
RUN apt-get update && apt-get install -y \
    software-properties-common \
    ssh \
    git \
    python3-pip \
    python3-vcstool \
    build-essential \
    cmake \
    python3-colcon-common-extensions \
    python3-flake8 \
    python3-pytest-cov \
    python3-rosdep \
    python3-setuptools \
    python3-vcstool \
    wget \
    ros-$ROSDISTRO-rqt \
    ros-foxy-rqt-topic \
    ros-foxy-rqt-top \
    ros-foxy-rqt-srv \
    ros-foxy-rqt-shell \
    ros-foxy-rqt-service-caller \
    ros-foxy-rqt-publisher \
    ros-foxy-rqt-reconfigure \
    ros-foxy-rqt-plot \
    ros-foxy-rqt-msg \
    ros-foxy-rqt-graph \
    ros-foxy-rqt-console \
    ros-foxy-rqt-common-plugins \
    ros-foxy-rqt-action \
    ros-$ROSDISTRO-rviz2 \
    python3-pykdl

RUN apt-get update && apt-get install -y \
    libbullet-dev \
    libasio-dev \
    libtinyxml2-dev \
    libcunit1-dev

#############################################
# Tesseract and SCXML deps from CRS project #
#############################################
# RUN apt-get update && apt-get install -y \
#     ros-$ROSDISTRO-vision-opencv \
#     ros-$ROSDISTRO-perception-pcl \
#     ros-$ROSDISTRO-octomap \
#     ros-$ROSDISTRO-ompl \
#     libboost-all-dev \
#     libboost-python-dev \
#     libyaml-cpp-dev \
#     libassimp-dev \
#     libeigen3-dev \
#     coinor-libipopt-dev \
#     bison \
#     && apt-add-repository ppa:beineri/opt-qt-5.14.2-focal && \
#     apt-get update && apt-get install -y \
#     qt514-meta-full

RUN pip3 install -U \
    argcomplete \
    flake8-blind-except \
    flake8-builtins \
    flake8-class-newline \
    flake8-comprehensions \
    flake8-deprecated \
    flake8-docstrings \
    flake8-import-order \
    flake8-quotes \
    pytest-repeat \
    pytest-rerunfailures \
    pytest \
    pyassimp==4.1.3 \
    gpiozero \
    pigpio


# Moveit 2 build process
RUN mkdir -p /moveit_ws/src
WORKDIR /moveit_ws/src
RUN ["/bin/bash", "-c", "source /opt/ros/foxy/setup.bash &&\
    git clone https://github.com/ros-planning/moveit2.git -b foxy &&\
    vcs-import < moveit2/moveit2.repos || true && \
    apt update && \
    rosdep install -r --from-paths . --ignore-src --rosdistro foxy -y"]

# reduce size by removing apt cache
RUN ["/bin/bash", "-c", "rm -rf /var/lib/apt/lists/*"]

WORKDIR /moveit_ws
RUN ["/bin/bash", "-c", "source /opt/ros/foxy/setup.bash &&\
    colcon build --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release"]

# # Tesseract build process
# RUN ["/bin/bash", "-c", "if["$TESSERACT" = "true"]; then \
#     mkdir -p /tesseract_ws/src &&\
#     cd /tesseract_ws &&\
#     source /opt/ros/$ROSDISTRO/setup.bash &&\
#     vcs import src < ./src/ram/ram.rosinstall &&\
#     vcs pull src || true &&\
#     colcon build --symlink-install --continue-on-error --cmake-args -Wno-dev -DBoost_DEBUG=ON -DBoost_NO_BOOST_CMAKE=ON; \
#     fi"]

# copy packages across
RUN mkdir -p /dev_ws/src/ram
COPY . /dev_ws/src/ram/

# RAM stack build 
WORKDIR /dev_ws
RUN ["/bin/bash", "-c", "source /opt/ros/$ROSDISTRO/setup.bash &&\
    source /moveit_ws/install/setup.bash &&\
    vcs import src < ./src/ram/dependencies.rosinstall &&\
    vcs pull src || true &&\
    colcon build --symlink-install \
    "]

RUN ["/bin/bash", "-c", "cp /dev_ws/src/ram/.docker/entrypoint.sh / && chmod 777 /entrypoint.sh"]
ENTRYPOINT [ "/entrypoint.sh" ]
CMD ["bash"]