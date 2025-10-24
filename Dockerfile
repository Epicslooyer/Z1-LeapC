# ================================================================
#  Base image: ROS Noetic desktop full (Ubuntu 20.04)
# ================================================================
FROM osrf/ros:noetic-desktop-full

# ------------------------------------------------
#  Build arguments (customizable at build time)
# ------------------------------------------------
ARG DEBIAN_FRONTEND=noninteractive
ARG ROS_DISTRO=noetic
ARG Z1_ROS_REPO=https://github.com/unitreerobotics/z1_ros.git
ARG Z1_ROS_VERSION=master
ARG PINOCCHIO_VERSION=v3.0.0
ENV PINOCCHIO_VERSION=${PINOCCHIO_VERSION}

# ------------------------------------------------
#  Environment setup
# ------------------------------------------------
ENV ROS_DISTRO=${ROS_DISTRO}
ENV CATKIN_WS=/root/z1_ws
ENV LEAP_SDK_ROOT=/opt/leap-sdk
# Define LD_LIBRARY_PATH without triggering undefined variable warnings
ENV LD_LIBRARY_PATH=/opt/leap-sdk/lib

SHELL ["/bin/bash", "-c"]

# ================================================================
# 1. Base system dependencies
# ================================================================
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    build-essential cmake git wget curl unzip sudo udev lsb-release \
    python3-pip python3-venv python3-dev python3-catkin-tools \
    python3-colcon-common-extensions python3-rospkg python3-empy \
    libusb-1.0-0-dev libgl1-mesa-glx libgl1-mesa-dri libglu1-mesa \
    libxi6 libxrandr2 libxinerama1 libxcursor1 libxext6 libxrender1 \
    libxtst6 libglfw3 libsm6 libxxf86vm1 libboost-all-dev libeigen3-dev \
    liburdfdom-dev libassimp-dev pkg-config \
    ros-${ROS_DISTRO}-ros-control ros-${ROS_DISTRO}-ros-controllers \
    ros-${ROS_DISTRO}-controller-manager ros-${ROS_DISTRO}-industrial-robot-client \
    ros-${ROS_DISTRO}-tf2-ros ros-${ROS_DISTRO}-tf-conversions \
    ros-${ROS_DISTRO}-moveit ros-${ROS_DISTRO}-joint-trajectory-controller \
    ros-${ROS_DISTRO}-trac-ik-kinematics-plugin \
    pybind11-dev && \
    rm -rf /var/lib/apt/lists/*

# Ensure Eigen headers are discoverable in /usr/local for older build scripts
RUN ln -sf /usr/include/eigen3/Eigen /usr/local/include/Eigen && \
    ln -sf /usr/include/eigen3/unsupported /usr/local/include/unsupported

# ================================================================
# 2. Python dependencies (installed separately for caching)
# ================================================================
RUN pip3 install --no-cache-dir \
    numpy opencv-python transforms3d catkin_pkg defusedxml


# 2.5 Python dependencies for pinnochio
RUN apt-get update && apt-get install -y \
    python3-numpy python3-pybind11 python3-setuptools python3-scipy \
    libassimp-dev libboost-all-dev cmake pkg-config && \
    rm -rf /var/lib/apt/lists/*

# Upgrade cmake to 3.22
RUN apt-get update && apt-get install -y wget && \
    wget -q https://github.com/Kitware/CMake/releases/download/v3.27.9/cmake-3.27.9-linux-x86_64.sh && \
    sh cmake-3.27.9-linux-x86_64.sh --skip-license --prefix=/usr && \
    rm cmake-3.27.9-linux-x86_64.sh && \
    cmake --version


# ================================================================
# 3. Build and install Pinocchio (Z1 dependency)
# ================================================================
RUN apt-get update && apt-get install -y \
    lsb-release gnupg2 curl && \
    curl -s https://robotpkg.openrobots.org/packages/debian/robotpkg.key | apt-key add - && \
    echo "deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" \
        > /etc/apt/sources.list.d/robotpkg.list && \
    apt-get update && apt-get install -y \
        python3-pip python3-pybind11 python3-numpy python3-scipy \
        libeigen3-dev libboost-all-dev liburdfdom-dev libassimp-dev \
        robotpkg-py38-pinocchio && \
    rm -rf /var/lib/apt/lists/*

RUN python3 -c "import pinocchio; print('Pinocchio version:', pinocchio.__version__)"

# ================================================================
# 4. Prepare Catkin workspace and clone Unitree Z1 ROS
# ================================================================
RUN mkdir -p ${CATKIN_WS}/src && \
    cd ${CATKIN_WS}/src && \
    git clone --recursive ${Z1_ROS_REPO} z1_ros && \
    cd z1_ros && git checkout ${Z1_ROS_VERSION}

# ================================================================
# 5. Resolve ROS dependencies
# ================================================================
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    (rosdep init || true) && rosdep update && \
    rosdep install --from-paths ${CATKIN_WS}/src --ignore-src -yr --rosdistro ${ROS_DISTRO}

# ================================================================
# 6. Build Catkin workspace
# ================================================================
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd ${CATKIN_WS} && \
    catkin config --extend /opt/ros/${ROS_DISTRO} --merge-devel \
                  --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    catkin build unitree_legged_msgs && \
    catkin build

# ================================================================
# 7. Leap SDK placeholders + bindings
# ================================================================
RUN mkdir -p ${LEAP_SDK_ROOT}/lib ${LEAP_SDK_ROOT}/include

COPY leapc-python-bindings /opt/leap/leapc-python-bindings
RUN pip3 install --no-cache-dir -e /opt/leap/leapc-python-bindings/leapc-python-api && \
    pip3 install --no-cache-dir -e /opt/leap/leapc-python-bindings/leapc-cffi

# ================================================================
# 8. Entrypoint setup
# ================================================================
COPY docker/ros_entrypoint.sh /usr/local/bin/ros_entrypoint.sh
RUN chmod +x /usr/local/bin/ros_entrypoint.sh

WORKDIR ${CATKIN_WS}

ENTRYPOINT ["/usr/local/bin/ros_entrypoint.sh"]
CMD ["bash"]
