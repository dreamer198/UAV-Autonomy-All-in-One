# shellcheck shell=bash
# shellcheck disable=SC1090
# ROS Noetic
[ -f /opt/ros/noetic/setup.bash ] && source /opt/ros/noetic/setup.bash

# Livox ROS1 workspace
[ -f ~/livox_ws/devel/setup.bash ] && source ~/livox_ws/devel/setup.bash

# Main catkin workspace
[ -f ~/catkin_ws/devel/setup.bash ] && source ~/catkin_ws/devel/setup.bash

# Real-flight defaults
export FCU_URL="${FCU_URL:-/dev/ttyACM0:921600}"
export DRONE_ID=0
# Network topology is site-specific; pass GCS_URL explicitly when telemetry
# forwarding is required instead of inheriting an obsolete workstation IP.
export GCS_URL="${GCS_URL:-}"
