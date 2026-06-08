#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME="${CONTAINER_NAME:-ros_noetic}"
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
JETSON_IP="${JETSON_IP:-10.0.30.108}"
JETSON_HOSTNAME="${JETSON_HOSTNAME:-jetson2-desktop}"
RVIZ_CONFIG_HOST="${RVIZ_CONFIG_HOST:-$PROJECT_ROOT/config/rviz/jetson_real_stack.rviz}"
RVIZ_CONFIG_CONTAINER="${RVIZ_CONFIG_CONTAINER:-/root/jetson_real_stack.rviz}"
GOAL_BRIDGE_HOST="${GOAL_BRIDGE_HOST:-$PROJECT_ROOT/scripts/rviz_goal_to_diff_planner.py}"
GOAL_BRIDGE_CONTAINER="${GOAL_BRIDGE_CONTAINER:-/root/rviz_goal_to_diff_planner.py}"
RVIZ_GOAL_Z="${RVIZ_GOAL_Z:-0.3}"
RVIZ_GOAL_FRAME="${RVIZ_GOAL_FRAME:-}"
DISPLAY_VALUE="${DISPLAY:-:0}"

detect_local_ip() {
  ip -4 addr show | awk '/inet 10\.0\.30\./ {print $2; exit}' | cut -d/ -f1
}

LOCAL_IP="${LOCAL_IP:-$(detect_local_ip)}"
if [ -z "$LOCAL_IP" ]; then
  echo "[ERROR] Could not detect local 10.0.30.x IP. Set LOCAL_IP manually."
  echo "Example: LOCAL_IP=10.0.30.196 $0"
  exit 1
fi

if ! command -v docker >/dev/null 2>&1; then
  echo "[ERROR] docker command not found."
  exit 1
fi

if ! docker inspect "$CONTAINER_NAME" >/dev/null 2>&1; then
  echo "[ERROR] Docker container '$CONTAINER_NAME' does not exist."
  exit 1
fi

if [ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER_NAME")" != "true" ]; then
  echo "[INFO] Starting container: $CONTAINER_NAME"
  docker start "$CONTAINER_NAME" >/dev/null
fi

if [ ! -f "$RVIZ_CONFIG_HOST" ]; then
  echo "[ERROR] RViz config not found: $RVIZ_CONFIG_HOST"
  exit 1
fi

if [ ! -f "$GOAL_BRIDGE_HOST" ]; then
  echo "[ERROR] RViz goal bridge not found: $GOAL_BRIDGE_HOST"
  exit 1
fi

if command -v xhost >/dev/null 2>&1; then
  xhost +local:docker >/dev/null 2>&1 || true
  xhost +SI:localuser:root >/dev/null 2>&1 || true
fi

docker cp "$RVIZ_CONFIG_HOST" "$CONTAINER_NAME:$RVIZ_CONFIG_CONTAINER"
docker cp "$GOAL_BRIDGE_HOST" "$CONTAINER_NAME:$GOAL_BRIDGE_CONTAINER"

docker exec "$CONTAINER_NAME" bash -lc "
set -e
grep -qE '(^|[[:space:]])$JETSON_HOSTNAME($|[[:space:]])' /etc/hosts 2>/dev/null || echo '$JETSON_IP $JETSON_HOSTNAME' >> /etc/hosts
perl -0pi -e 's/\\n?# >>> jetson_ros1_rviz\\n.*?\\n# <<< jetson_ros1_rviz\\n?/\\n/s' ~/.bashrc
{
  echo '# >>> jetson_ros1_rviz'
  echo 'export ROS_MASTER_URI=http://$JETSON_IP:11311'
  echo 'export ROS_IP=$LOCAL_IP'
  echo 'unset ROS_HOSTNAME'
  echo \"grep -qE '(^|[[:space:]])$JETSON_HOSTNAME($|[[:space:]])' /etc/hosts 2>/dev/null || echo '$JETSON_IP $JETSON_HOSTNAME' >> /etc/hosts\"
  echo '# <<< jetson_ros1_rviz'
} >> ~/.bashrc
"

docker exec "$CONTAINER_NAME" bash -lc "
source /opt/ros/noetic/setup.bash
source ~/.bashrc
rosnode kill /jetson_rviz_world_camera_init_tf >/dev/null 2>&1 || true
rosnode kill /jetson_rviz_world_map_tf >/dev/null 2>&1 || true
rosnode kill /rviz_goal_to_diff_planner >/dev/null 2>&1 || true
" >/dev/null 2>&1 || true

docker exec -d "$CONTAINER_NAME" bash -lc "
source /opt/ros/noetic/setup.bash
source ~/.bashrc
exec rosrun tf static_transform_publisher 0 0 0 0 0 0 world camera_init 100 __name:=jetson_rviz_world_camera_init_tf
"

docker exec -d "$CONTAINER_NAME" bash -lc "
source /opt/ros/noetic/setup.bash
source ~/.bashrc
exec rosrun tf static_transform_publisher 0 0 0 0 0 0 world map 100 __name:=jetson_rviz_world_map_tf
"

BRIDGE_ARGS="_default_z:=$RVIZ_GOAL_Z _output_topic:=/goal"
if [ -n "$RVIZ_GOAL_FRAME" ]; then
  BRIDGE_ARGS="$BRIDGE_ARGS _frame_id:=$RVIZ_GOAL_FRAME"
fi

docker exec -d "$CONTAINER_NAME" bash -lc "
source /opt/ros/noetic/setup.bash
source ~/.bashrc
exec python3 '$GOAL_BRIDGE_CONTAINER' $BRIDGE_ARGS
"

echo "[INFO] Starting RViz in container '$CONTAINER_NAME'"
echo "[INFO] ROS_MASTER_URI=http://$JETSON_IP:11311"
echo "[INFO] ROS_IP=$LOCAL_IP"
echo "[INFO] RViz goal bridge: /move_base_simple/goal and /clicked_point -> /goal"
echo "[INFO] 2D Nav Goal fixed height: RVIZ_GOAL_Z=$RVIZ_GOAL_Z"

exec docker exec -it \
  -e DISPLAY="$DISPLAY_VALUE" \
  -e QT_X11_NO_MITSHM=1 \
  "$CONTAINER_NAME" \
  bash -lc "source ~/.bashrc && rviz -d '$RVIZ_CONFIG_CONTAINER'"
