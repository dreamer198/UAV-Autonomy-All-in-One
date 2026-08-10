# shellcheck shell=bash disable=SC1090,SC1091
# ROS Noetic environment for the display-and-command ground station only.
[ -f /opt/ros/noetic/setup.bash ] && source /opt/ros/noetic/setup.bash
[ -f /root/ground_station_ws/devel/setup.bash ] && \
  source /root/ground_station_ws/devel/setup.bash
