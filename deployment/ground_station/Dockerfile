FROM ros:noetic-ros-base-focal

LABEL org.opencontainers.image.title="UAV Autonomy All-in-One ground station" \
      io.uav-autonomy-aio.ground-station-layout="v1"

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=noetic \
    ROS_EDITION=ROS1 \
    SIM2REAL_RUNTIME_MODE=ground_station

SHELL ["/bin/bash", "-lc"]

# The ground station deliberately excludes Livox, FAST-LIO, planner backends,
# controllers, PX4 device access, and the real-flight runtime.  Only the
# MAVROS messages are installed for the guarded RViz goal bridge.
RUN apt-get -o Acquire::Retries=5 update \
 && apt-get install -y --no-install-recommends \
    python3-catkin-tools \
    ros-noetic-mavros-msgs \
    ros-noetic-message-generation \
    ros-noetic-rviz \
    ros-noetic-std-srvs \
 && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /root/ground_station_ws/src
COPY planning/ros_pkgs/sim2real_planning_msgs \
     /root/ground_station_ws/src/sim2real_planning_msgs

RUN source /opt/ros/noetic/setup.bash \
 && cd /root/ground_station_ws \
 && catkin init \
 && catkin config \
      --extend /opt/ros/noetic \
      --cmake-args -DCMAKE_BUILD_TYPE=Release \
 && catkin build --no-status sim2real_planning_msgs

COPY deployment/ground_station/root.bashrc /root/.bashrc

RUN source /root/.bashrc \
 && command -v rviz >/dev/null \
 && command -v rosservice >/dev/null \
 && rospack find sim2real_planning_msgs >/dev/null \
 && python3 -c 'from mavros_msgs.msg import State; from sim2real_planning_msgs.msg import PlannerGoal, PlannerStatus; from sim2real_planning_msgs.srv import ValidateGoal; from std_srvs.srv import Trigger'

WORKDIR /root
CMD ["tail", "-f", "/dev/null"]
