ARG ROSDISTRO=humble

FROM gdwyer/ros:$ROSDISTRO

ARG ROSDISTRO
ARG MOVEIT2=true
ARG TESSERACT=false

ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV DEBIAN_FRONTEND=noninteractive


# Install any needed packages then clean up apt cache
# Build tools and general ROS tools
RUN apt-get update && apt-get install -y \
    libnlopt-cxx-dev \
    libbullet-dev \
    libasio-dev \
    libtinyxml2-dev \
    libcunit1-dev \
    qtbase5-dev \
    libqt5svg5-dev \
    libzmq3-dev \
    libdw-dev \
    libboost-dev \
    ros-$ROSDISTRO-example-interfaces

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
    pyassimp==4.1.3 \
    gpiozero \
    pigpio

# reduce size by removing apt cache
RUN ["/bin/bash", "-c", "rm -rf /var/lib/apt/lists/*"]

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
    colcon build --symlink-install || true\
    "]

RUN ["/bin/bash", "-c", "cp /dev_ws/src/ram/.docker/entrypoint.sh / && chmod 777 /entrypoint.sh"]
ENTRYPOINT [ "/entrypoint.sh" ]
CMD ["bash"]