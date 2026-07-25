FROM ros:noetic-ros-base-focal

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=noetic \
    ROS_EDITION=ROS1

SHELL ["/bin/bash", "-lc"]

ARG UBUNTU_PORTS_MIRROR=http://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports

RUN sed -i "s|http://ports.ubuntu.com/ubuntu-ports|${UBUNTU_PORTS_MIRROR}|g" /etc/apt/sources.list \
 && apt-get -o Acquire::Retries=5 update \
 && apt-get install -y \
    build-essential \
    cmake \
    curl \
    git \
    iproute2 \
    iputils-ping \
    libapr1-dev \
    libarmadillo-dev \
    libeigen3-dev \
    libgflags-dev \
    libgoogle-glog-dev \
    libopencv-dev \
    libpcl-dev \
    net-tools \
    nano \
    pcl-tools \
    pkg-config \
    python3-catkin-tools \
    python3-osrf-pycommon \
    python3-pip \
    python3-rosdep \
    ros-noetic-cv-bridge \
    ros-noetic-ddynamic-reconfigure \
    ros-noetic-eigen-conversions \
    ros-noetic-mavros \
    ros-noetic-mavros-extras \
    ros-noetic-pcl-ros \
    ros-noetic-roslint \
    ros-noetic-rviz \
    ros-noetic-tf \
    ros-noetic-tf2-geometry-msgs \
    ros-noetic-tf2-ros \
    ros-noetic-tf2-sensor-msgs \
    usbutils \
    vim \
    wget \
 && rm -rf /var/lib/apt/lists/*

RUN /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh

WORKDIR /root

RUN mkdir -p \
    /root/code \
    /root/flight_bags \
    /root/livox_ws/src \
    /root/catkin_ws/src

COPY deployment/docker/root.bashrc /root/.bashrc
COPY deployment/docker/livox_ros_driver2.package.ros1.xml /tmp/livox_ros_driver2.package.xml
COPY deployment/docker/patch_fastlio_for_livox_ros_driver2.sh /tmp/patch_fastlio_for_livox_ros_driver2.sh

COPY third_party/Livox-SDK2 /tmp/src/Livox-SDK2
COPY third_party/livox_ros_driver2 /root/livox_ws/src/livox_ros_driver2
COPY third_party/FAST_LIO /root/catkin_ws/src/FAST_LIO
COPY third_party/Diff-Planner-PX4 /root/catkin_ws/src/Diff-Planner-PX4
COPY common /root/catkin_ws/src/sim2real_common
COPY deployment/ros_pkgs/sim2real_deployment /root/catkin_ws/src/sim2real_deployment

COPY deployment/config/rviz/jetson_real_stack.rviz /root/jetson_real_stack.rviz
COPY deployment/config/livox/MID360s_config.json /root/livox_ws/src/livox_ros_driver2/config/MID360s_config.json
COPY deployment/config/controller.yaml /root/deployment/controller.yaml

RUN chmod +x \
    /tmp/patch_fastlio_for_livox_ros_driver2.sh \
    /root/catkin_ws/src/sim2real_common/scripts/*.py \
    /root/catkin_ws/src/sim2real_deployment/scripts/*.py \
 && cp /tmp/livox_ros_driver2.package.xml /root/livox_ws/src/livox_ros_driver2/package.xml \
 && /tmp/patch_fastlio_for_livox_ros_driver2.sh /root/catkin_ws/src/FAST_LIO

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
 && cd /root/catkin_ws \
 && catkin init \
 && catkin config --extend /root/livox_ws/devel --cmake-args -DCMAKE_BUILD_TYPE=Release \
 && catkin build quadrotor_msgs pose_utils traj_utils sim2real_common sim2real_deployment \
 && catkin build

WORKDIR /root

CMD ["bash"]
