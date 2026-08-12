FROM ros:noetic-ros-base-focal

LABEL org.opencontainers.image.title="UAV Autonomy All-in-One real-flight runtime" \
      io.sim2real.planner-workspaces="v2"

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=noetic \
    ROS_EDITION=ROS1 \
    SIM2REAL_RUNTIME_MODE=real

SHELL ["/bin/bash", "-lc"]

ARG UBUNTU_PORTS_MIRROR=http://mirrors.aliyun.com/ubuntu-ports
ARG NLOPT_DEBIAN_VERSION=2.6.1-8ubuntu2

RUN sed -i "s|http://ports.ubuntu.com/ubuntu-ports|${UBUNTU_PORTS_MIRROR}|g" /etc/apt/sources.list \
 && apt-get -o Acquire::Retries=5 update \
 && apt-get -o Acquire::Retries=5 --fix-missing install -y \
    build-essential \
    cmake \
    curl \
    git \
    iproute2 \
    iputils-ping \
    libapr1-dev \
    libarmadillo-dev \
    libdw-dev \
    libeigen3-dev \
    libgflags-dev \
    libgoogle-glog-dev \
    libnlopt-cxx-dev="${NLOPT_DEBIAN_VERSION}" \
    libnlopt-dev="${NLOPT_DEBIAN_VERSION}" \
    libopencv-dev \
    libpcl-dev \
    libyaml-cpp-dev \
    net-tools \
    nano \
    pcl-tools \
    pkg-config \
    python3-catkin-tools \
    python3-osrf-pycommon \
    python3-pip \
    python3-rosdep \
    python3-yaml \
    rsync \
    ros-noetic-cv-bridge \
    ros-noetic-actionlib \
    ros-noetic-actionlib-msgs \
    ros-noetic-ddynamic-reconfigure \
    ros-noetic-dynamic-reconfigure \
    ros-noetic-eigen-conversions \
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
    usbutils \
    vim \
    wget \
 && rm -rf /var/lib/apt/lists/*

RUN /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh

WORKDIR /root

RUN mkdir -p \
    /opt/uav-autonomy-aio \
    /root/code \
    /root/flight_bags \
    /root/livox_ws/src \
    /root/localization_ws/src

COPY deployment/docker/root.bashrc /root/.bashrc
COPY deployment/docker/livox_ros_driver2.package.ros1.xml /tmp/livox_ros_driver2.package.xml
COPY deployment/docker/patch_fastlio_for_livox_ros_driver2.sh /tmp/patch_fastlio_for_livox_ros_driver2.sh

COPY third_party/Livox-SDK2 /tmp/src/Livox-SDK2
COPY third_party/livox_ros_driver2 /root/livox_ws/src/livox_ros_driver2
COPY third_party/FAST_LIO /root/localization_ws/src/FAST_LIO
COPY third_party/Diff-Planner-PX4 /opt/uav-autonomy-aio/third_party/Diff-Planner-PX4
COPY third_party/Fast-Planner /opt/uav-autonomy-aio/third_party/Fast-Planner
COPY third_party/SUPER /opt/uav-autonomy-aio/third_party/SUPER
COPY common /opt/uav-autonomy-aio/common
COPY deployment/ros_pkgs/sim2real_deployment \
     /opt/uav-autonomy-aio/deployment/ros_pkgs/sim2real_deployment
COPY planning /opt/uav-autonomy-aio/planning

COPY deployment/config/rviz/jetson_real_stack.rviz /root/jetson_real_stack.rviz
COPY deployment/config/livox/MID360s_config.json /root/livox_ws/src/livox_ros_driver2/config/MID360s_config.json
COPY deployment/config/controller.yaml /root/deployment/controller.yaml

RUN chmod +x \
    /tmp/patch_fastlio_for_livox_ros_driver2.sh \
    /opt/uav-autonomy-aio/planning/scripts/*.sh \
    /opt/uav-autonomy-aio/common/scripts/*.py \
    /opt/uav-autonomy-aio/deployment/ros_pkgs/sim2real_deployment/scripts/*.py \
 && cp /tmp/livox_ros_driver2.package.xml /root/livox_ws/src/livox_ros_driver2/package.xml \
 && /tmp/patch_fastlio_for_livox_ros_driver2.sh /root/localization_ws/src/FAST_LIO

RUN source /opt/ros/noetic/setup.bash \
 && echo "Skip rosdep install: real-flight dependencies are installed explicitly with apt."

RUN cd /tmp/src/Livox-SDK2 \
 && rm -rf build \
 && mkdir -p build \
 && cd build \
 && cmake .. \
 && make -j"$(nproc)" \
 && make install

RUN source /opt/ros/noetic/setup.bash \
 && cd /root/livox_ws \
 && catkin init \
 && catkin config --extend /opt/ros/noetic --cmake-args -DROS_EDITION=ROS1 -DCMAKE_BUILD_TYPE=Release \
 && catkin build livox_ros_driver2

RUN source /opt/ros/noetic/setup.bash \
 && source /root/livox_ws/devel/setup.bash \
 && cd /root/localization_ws \
 && catkin init \
 && catkin config --extend /root/livox_ws/devel --cmake-args -DCMAKE_BUILD_TYPE=Release \
 && catkin build

# Build public/control, Diff, Fast, and SUPER in independent catkin domains. Only the
# interface and control setup files are sourced by /root/.bashrc at runtime.
ARG BUILD_JOBS=2
RUN /opt/uav-autonomy-aio/planning/scripts/build_planner_workspaces.sh \
      --project-root /opt/uav-autonomy-aio \
      --workspace-root /opt/uav-autonomy-aio/planning/workspaces \
      --underlay /root/localization_ws/devel \
      --flavor deployment \
      --jobs "$BUILD_JOBS" \
 && mkdir -p /root/catkin_ws/devel \
 && ln -s /opt/uav-autonomy-aio/planning/workspaces/control_ws/devel/setup.bash \
      /root/catkin_ws/devel/setup.bash

WORKDIR /root

CMD ["bash"]
