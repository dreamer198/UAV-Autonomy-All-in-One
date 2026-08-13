FROM ros:noetic-ros-base-focal

LABEL org.opencontainers.image.title="UAV Autonomy All-in-One ground station" \
      io.uav-autonomy-aio.ground-station-layout="v2"

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=noetic \
    ROS_EDITION=ROS1 \
    SIM2REAL_RUNTIME_MODE=ground_station

SHELL ["/bin/bash", "-lc"]

ARG UBUNTU_MIRROR=https://mirrors.aliyun.com/ubuntu

# The ground station deliberately excludes Livox, FAST-LIO, planner backends,
# controllers, PX4 device access, and the real-flight runtime.  Only the
# MAVROS messages are installed for the guarded RViz goal bridge.
RUN sed -i \
      -e "s|http://archive.ubuntu.com/ubuntu|${UBUNTU_MIRROR}|g" \
      -e "s|http://security.ubuntu.com/ubuntu|${UBUNTU_MIRROR}|g" \
      /etc/apt/sources.list \
 && apt-get -o Acquire::Retries=5 update \
 && apt-get install -y --no-install-recommends \
    python3-catkin-tools \
    python3-numpy \
    qtbase5-dev \
    ros-noetic-actionlib \
    ros-noetic-actionlib-msgs \
    ros-noetic-mavros-msgs \
    ros-noetic-message-generation \
    ros-noetic-nav-msgs \
    ros-noetic-pluginlib \
    ros-noetic-rviz \
    ros-noetic-sensor-msgs \
    ros-noetic-std-srvs \
    ros-noetic-tf2-ros \
    ros-noetic-visualization-msgs \
 && rm -rf /var/lib/apt/lists/*

# Keep the GUI dependency layer reusable when the CJK font choice changes.
# Micro Hei covers Simplified Chinese while adding only a small amount to the
# disposable ground-station image.
RUN apt-get -o Acquire::Retries=5 update \
 && apt-get install -y --no-install-recommends fonts-wqy-microhei \
 && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /root/ground_station_ws/src
COPY planning/ros_pkgs/sim2real_planning_msgs \
     /root/ground_station_ws/src/sim2real_planning_msgs
COPY deployment/ros_pkgs/sim2real_ground_station \
     /root/ground_station_ws/src/sim2real_ground_station

RUN source /opt/ros/noetic/setup.bash \
 && cd /root/ground_station_ws \
 && catkin init \
 && catkin config \
      --extend /opt/ros/noetic \
      --cmake-args -DCMAKE_BUILD_TYPE=Release \
 && catkin build --no-status sim2real_planning_msgs sim2real_ground_station

COPY deployment/ground_station/root.bashrc /root/.bashrc

RUN source /root/.bashrc \
 && command -v rviz >/dev/null \
 && command -v rosservice >/dev/null \
 && fc-match -f '%{family}\n' 'WenQuanYi Micro Hei:lang=zh-cn' | grep -q 'WenQuanYi Micro Hei' \
 && rospack find sim2real_planning_msgs >/dev/null \
 && rospack find sim2real_ground_station >/dev/null \
 && python3 -c 'import numpy, tf2_ros; from rviz import bindings as rviz; assert rviz.VisualizationFrame; from mavros_msgs.msg import ExtendedState, State; from nav_msgs.msg import Odometry; from sensor_msgs import point_cloud2; from sensor_msgs.msg import BatteryState, NavSatFix, PointCloud2; from sim2real_planning_msgs.msg import FlightCommandAction, InteractiveGoalAction, PlannerGoal, PlannerStatus; from sim2real_planning_msgs.srv import ValidateGoal; from visualization_msgs.msg import Marker' \
 && test -x /root/ground_station_ws/devel/lib/sim2real_ground_station/ground_station_telemetry.py

WORKDIR /root
CMD ["tail", "-f", "/dev/null"]
