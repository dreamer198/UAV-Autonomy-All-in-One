# ROS Noetic
[ -f /opt/ros/noetic/setup.bash ] && source /opt/ros/noetic/setup.bash

# Livox ROS1 workspace
[ -f ~/livox_ws/devel/setup.bash ] && source ~/livox_ws/devel/setup.bash

# Main catkin workspace
[ -f ~/catkin_ws/devel/setup.bash ] && source ~/catkin_ws/devel/setup.bash

# Real-flight defaults
export FCU_URL="${FCU_URL:-/dev/ttyACM0:921600}"
export DRONE_ID="${DRONE_ID:-0}"
export GCS_URL="${GCS_URL:-udp://:14555@10.0.30.196:14550}"
