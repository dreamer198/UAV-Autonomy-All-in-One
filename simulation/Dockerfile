# syntax=docker/dockerfile:1
FROM osrf/ros:noetic-desktop-full

ARG PX4_REPOSITORY=https://github.com/PX4/PX4-Autopilot.git
ARG PX4_TAG=v1.14.3
ARG PX4_COMMIT=1dacb4cdef2d7145754fc788fa8dc482eed74b40
ARG MID360_REPOSITORY=https://github.com/Tfly6/Mid360_px4_sim_plugin.git
ARG MID360_COMMIT=44bcd80e0edf6e797edd004cd5bb2bb8fcfd9d18
ARG BUILD_JOBS=4
ARG NLOPT_DEBIAN_VERSION=2.6.1-8ubuntu2

LABEL org.opencontainers.image.title="UAV Autonomy All-in-One simulation" \
      io.uav-autonomy-aio.simulation.px4-commit="$PX4_COMMIT" \
      io.uav-autonomy-aio.simulation.mid360-commit="$MID360_COMMIT" \
      io.sim2real.planner-workspaces="v2"

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=noetic \
    PX4_DIR=/opt/PX4-Autopilot \
    SIMULATION_WS=/opt/simulation_ws \
    SIM2REAL_RUNTIME_MODE=simulation \
    PYTHONDONTWRITEBYTECODE=1

SHELL ["/bin/bash", "-lc"]

RUN apt-get -o Acquire::Retries=5 update \
 && apt-get install -y --no-install-recommends \
    astyle \
    build-essential \
    cmake \
    cppcheck \
    curl \
    file \
    g++ \
    gcc \
    gdb \
    git \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    iproute2 \
    iputils-ping \
    lcov \
    libapr1-dev \
    libarmadillo-dev \
    libdw-dev \
    libeigen3-dev \
    libgazebo11-dev \
    libgflags-dev \
    libgoogle-glog-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer1.0-dev \
    libnlopt-cxx-dev="${NLOPT_DEBIAN_VERSION}" \
    libnlopt-dev="${NLOPT_DEBIAN_VERSION}" \
    libopencv-dev \
    libpcl-dev \
    libprotobuf-dev \
    libxml2-dev \
    libxml2-utils \
    libyaml-cpp-dev \
    net-tools \
    ninja-build \
    pkg-config \
    protobuf-compiler \
    python3-catkin-tools \
    python3-dev \
    python3-empy \
    python3-numpy \
    python3-osrf-pycommon \
    python3-pip \
    python3-rosdep \
    python3-setuptools \
    python3-wheel \
    python3-yaml \
    rsync \
    ros-noetic-cv-bridge \
    ros-noetic-ddynamic-reconfigure \
    ros-noetic-dynamic-reconfigure \
    ros-noetic-eigen-conversions \
    ros-noetic-gazebo-plugins \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-laser-geometry \
    ros-noetic-mavros \
    ros-noetic-mavros-extras \
    ros-noetic-message-filters \
    ros-noetic-pcl-ros \
    ros-noetic-roslint \
    ros-noetic-rviz \
    ros-noetic-tf \
    ros-noetic-tf2-geometry-msgs \
    ros-noetic-tf2-ros \
    ros-noetic-tf2-sensor-msgs \
    ros-noetic-visualization-msgs \
    shellcheck \
    sudo \
    unzip \
    vim \
    wget \
    zip \
 && rm -rf /var/lib/apt/lists/*

# PX4 v1.14 and ROS Noetic generators both use the Empy 3 API. 3.3.2 is the
# Focal/Noetic version; 3.3.4 breaks gencpp by shadowing its len() template
# helper, while Empy 4 removes the API entirely.
RUN python3 -m pip install --no-cache-dir "empy==3.3.2"

RUN git clone --branch "$PX4_TAG" --depth 1 "$PX4_REPOSITORY" "$PX4_DIR" \
 && test "$(git -C "$PX4_DIR" rev-parse HEAD)" = "$PX4_COMMIT" \
 && git -C "$PX4_DIR" submodule update --init --depth 1 --recursive \
      Tools/simulation/gazebo-classic/sitl_gazebo-classic \
      src/drivers/cyphal/legacy_data_types \
      src/drivers/cyphal/libcanard \
      src/drivers/cyphal/public_regulated_data_types \
      src/drivers/gps/devices \
      src/drivers/uavcan/libuavcan \
      src/lib/crypto/libtomcrypt \
      src/lib/crypto/libtommath \
      src/lib/crypto/monocypher \
      src/lib/events/libevents \
      src/modules/mavlink/mavlink \
      src/modules/uxrce_dds_client/Micro-XRCE-DDS-Client

RUN python3 -m pip install --no-cache-dir -r "$PX4_DIR/Tools/setup/requirements.txt" \
 && python3 -m pip install --no-cache-dir "numpy==1.23.5" \
 && python3 -m pip check

RUN mkdir -p "$SIMULATION_WS/src" \
 && git clone "$MID360_REPOSITORY" "$SIMULATION_WS/src/Mid360_px4_sim_plugin" \
 && git -C "$SIMULATION_WS/src/Mid360_px4_sim_plugin" checkout --detach "$MID360_COMMIT" \
 && test "$(git -C "$SIMULATION_WS/src/Mid360_px4_sim_plugin" rev-parse HEAD)" = "$MID360_COMMIT"

# Keep the local assets that used to live only in ros_noetic's writable /root
# mount under version control and apply them to the pinned PX4 tree.
COPY simulation/assets/px4/launch/outdoor_mid360.launch "$PX4_DIR/launch/outdoor_mid360.launch"
COPY simulation/assets/px4/launch/px4_config.yaml "$PX4_DIR/launch/px4_config.yaml"
COPY simulation/assets/px4/worlds/ego_swarm.world "$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/ego_swarm.world"
COPY simulation/assets/px4/models/iris_mid360/ "$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris_mid360/"

RUN cp -a "$SIMULATION_WS/src/Mid360_px4_sim_plugin/livox_laser_simulation/models/Mid360" \
      "$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/Mid360" \
 && source /opt/ros/noetic/setup.bash \
 && cd "$SIMULATION_WS" \
 && catkin init \
 && catkin config --extend /opt/ros/noetic --cmake-args -DCMAKE_BUILD_TYPE=Release \
 && catkin build --no-status -j"$BUILD_JOBS" -p"$BUILD_JOBS" livox_laser_simulation

RUN cd "$PX4_DIR" \
 && make -j"$BUILD_JOBS" px4_sitl_default

# Build Gazebo only after the PX4 target has generated the MAVLink headers
# pinned by the PX4 source tree. Building both make goals in parallel races
# with /opt/ros/noetic's newer MAVLink headers and is not reproducible.
RUN cd "$PX4_DIR" \
 && make -j"$BUILD_JOBS" px4_sitl_default sitl_gazebo-classic

RUN /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh

COPY simulation/root.bashrc /root/.bashrc

# Keep the upstream scanner geometry and scan pattern, but disable rendering
# thousands of Gazebo ray lines. PointCloud2 generation remains unchanged.
COPY simulation/assets/px4/models/Mid360/Mid360.sdf \
     "$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/Mid360/Mid360.sdf"

RUN source /root/.bashrc \
 && test "$(realpath "$(rospack find px4)")" = "$PX4_DIR" \
 && test "$(realpath "$(rospack find livox_laser_simulation)")" = "$SIMULATION_WS/src/Mid360_px4_sim_plugin/livox_laser_simulation" \
 && test -x "$PX4_DIR/build/px4_sitl_default/bin/px4" \
 && test -e "$SIMULATION_WS/devel/lib/liblivox_laser_simulation.so"

WORKDIR /root
CMD ["tail", "-f", "/dev/null"]
