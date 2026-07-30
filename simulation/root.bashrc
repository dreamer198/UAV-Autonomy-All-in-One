# shellcheck shell=bash disable=SC1090,SC1091
# ROS Noetic + repository-owned PX4/Gazebo/Mid360 simulation environment.
[ -f /opt/ros/noetic/setup.bash ] && source /opt/ros/noetic/setup.bash
[ -f /opt/simulation_ws/devel/setup.bash ] && source /opt/simulation_ws/devel/setup.bash

# Source only the public interfaces and control plane globally. Planner-native
# workspaces remain isolated and are sourced by the manager's backend subprocess.
SIM2REAL_PROJECT_ROOT="${SIM2REAL_PROJECT_ROOT:-/opt/uav-autonomy-aio}"
SIM2REAL_INTERFACES_SETUP="$SIM2REAL_PROJECT_ROOT/planning/workspaces/interfaces_ws/devel/setup.bash"
SIM2REAL_CONTROL_SETUP="$SIM2REAL_PROJECT_ROOT/planning/workspaces/control_ws/devel/setup.bash"
if [ -f "$SIM2REAL_INTERFACES_SETUP" ]; then
  source "$SIM2REAL_INTERFACES_SETUP"
fi
if [ -f "$SIM2REAL_CONTROL_SETUP" ]; then
  source "$SIM2REAL_CONTROL_SETUP"
else
  # Compatibility fallback for a container created before workspace isolation.
  SIM2REAL_LEGACY_WS="${SIM_WORKSPACE_CONTAINER:-/workspaces/sim2real_ws}"
  [ ! -f "$SIM2REAL_LEGACY_WS/devel/setup.bash" ] ||
    source "$SIM2REAL_LEGACY_WS/devel/setup.bash"
fi

PX4_DIR=/opt/PX4-Autopilot
PX4_BUILD_DIR="$PX4_DIR/build/px4_sitl_default"
PX4_GAZEBO_DIR="$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic"

if [ -f "$PX4_DIR/Tools/simulation/gazebo-classic/setup_gazebo.bash" ]; then
  source "$PX4_DIR/Tools/simulation/gazebo-classic/setup_gazebo.bash" \
    "$PX4_DIR" "$PX4_BUILD_DIR" >/dev/null
fi

export ROS_PACKAGE_PATH="${ROS_PACKAGE_PATH:-}:$PX4_DIR:$PX4_GAZEBO_DIR"
export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH:-}:$PX4_GAZEBO_DIR/models"
export GAZEBO_PLUGIN_PATH="/opt/simulation_ws/devel/lib:${GAZEBO_PLUGIN_PATH:-}:$PX4_BUILD_DIR/build_gazebo-classic"
export LD_LIBRARY_PATH="/opt/simulation_ws/devel/lib:${LD_LIBRARY_PATH:-}:$PX4_BUILD_DIR/build_gazebo-classic"
