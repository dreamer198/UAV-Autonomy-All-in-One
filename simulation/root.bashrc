# shellcheck shell=bash disable=SC1091
# ROS Noetic + repository-owned PX4/Gazebo/Mid360 simulation environment.
[ -f /opt/ros/noetic/setup.bash ] && source /opt/ros/noetic/setup.bash
[ -f /opt/simulation_ws/devel/setup.bash ] && source /opt/simulation_ws/devel/setup.bash

# The deployment overlay is mounted at runtime and may not exist while the
# image is being built. Source it before appending the non-catkin PX4 paths,
# because catkin setup files replace ROS_PACKAGE_PATH.
SIM2REAL_WS="${SIM_WORKSPACE_CONTAINER:-/workspaces/sim2real_ws}"
if [ -f "$SIM2REAL_WS/devel/setup.bash" ]; then
  source "$SIM2REAL_WS/devel/setup.bash"
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
