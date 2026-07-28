# shellcheck shell=bash
# shellcheck disable=SC1090,SC1091
# ROS Noetic
[ -f /opt/ros/noetic/setup.bash ] && source /opt/ros/noetic/setup.bash

# Livox ROS1 workspace
[ -f ~/livox_ws/devel/setup.bash ] && source ~/livox_ws/devel/setup.bash

# Localization workspace contains FAST-LIO only.
[ -f ~/localization_ws/devel/setup.bash ] && source ~/localization_ws/devel/setup.bash

# Source the neutral interface and control workspaces globally. Planner-native
# packages are deliberately absent here and are sourced by the selected
# backend subprocess only.
SIM2REAL_PROJECT_ROOT="${SIM2REAL_PROJECT_ROOT:-/opt/uav-autonomy-aio}"
[ ! -f "$SIM2REAL_PROJECT_ROOT/planning/workspaces/interfaces_ws/devel/setup.bash" ] ||
  source "$SIM2REAL_PROJECT_ROOT/planning/workspaces/interfaces_ws/devel/setup.bash"
if [ -f "$SIM2REAL_PROJECT_ROOT/planning/workspaces/control_ws/devel/setup.bash" ]; then
  source "$SIM2REAL_PROJECT_ROOT/planning/workspaces/control_ws/devel/setup.bash"
else
  # Compatibility fallback for older real-flight images.
  [ -f ~/catkin_ws/devel/setup.bash ] && source ~/catkin_ws/devel/setup.bash
fi

# Real-flight defaults
export FCU_URL="${FCU_URL:-/dev/ttyACM0:921600}"
export DRONE_ID=0
# Network topology is site-specific; pass GCS_URL explicitly when telemetry
# forwarding is required instead of inheriting an obsolete workstation IP.
export GCS_URL="${GCS_URL:-}"
