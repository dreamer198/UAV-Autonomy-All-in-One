#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME="${CONTAINER_NAME:-ros_noetic}"
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
JETSON_IP="${JETSON_IP:-10.0.30.108}"
JETSON_HOSTNAME="${JETSON_HOSTNAME:-jetson2-desktop}"
RVIZ_CONFIG_HOST="${RVIZ_CONFIG_HOST:-$PROJECT_ROOT/deployment/config/rviz/jetson_real_stack.rviz}"
RVIZ_CONFIG_CONTAINER="${RVIZ_CONFIG_CONTAINER:-/root/jetson_real_stack.rviz}"
GOAL_BRIDGE_HOST="${GOAL_BRIDGE_HOST:-$PROJECT_ROOT/common/scripts/rviz_goal_to_diff_planner.py}"
GOAL_BRIDGE_CONTAINER="${GOAL_BRIDGE_CONTAINER:-/root/rviz_goal_to_diff_planner.py}"
START_GOAL_BRIDGE="${START_GOAL_BRIDGE:-true}"
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

REMOTE_ROS_MASTER_URI="http://$JETSON_IP:11311"
ROS_DOCKER_ENV=(
  -e "JETSON_ROS_MASTER_URI=$REMOTE_ROS_MASTER_URI"
  -e "JETSON_ROS_IP=$LOCAL_IP"
)

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

if [ "$START_GOAL_BRIDGE" != "true" ] && [ "$START_GOAL_BRIDGE" != "false" ]; then
  echo "[ERROR] START_GOAL_BRIDGE must be true or false."
  exit 1
fi

if [ "$START_GOAL_BRIDGE" = "true" ] && [ ! -f "$GOAL_BRIDGE_HOST" ]; then
  echo "[ERROR] RViz goal bridge not found: $GOAL_BRIDGE_HOST"
  exit 1
fi

if command -v xhost >/dev/null 2>&1; then
  xhost +local:docker >/dev/null 2>&1 || true
  xhost +SI:localuser:root >/dev/null 2>&1 || true
fi

docker cp "$RVIZ_CONFIG_HOST" "$CONTAINER_NAME:$RVIZ_CONFIG_CONTAINER"
if [ "$START_GOAL_BRIDGE" = "true" ]; then
  docker cp "$GOAL_BRIDGE_HOST" "$CONTAINER_NAME:$GOAL_BRIDGE_CONTAINER"
fi

docker exec "$CONTAINER_NAME" bash -lc '
set -e
jetson_ip="$1"
jetson_hostname="$2"

hosts_tmp="$(mktemp)"
mapping_updated=0
while IFS= read -r line || [ -n "$line" ]; do
  content="$line"
  comment=""
  if [[ "$line" == *#* ]]; then
    content="${line%%#*}"
    comment="#${line#*#}"
  fi

  read -r -a fields <<< "$content"
  hostname_found=0
  for ((i = 1; i < ${#fields[@]}; i++)); do
    if [ "${fields[$i]}" = "$jetson_hostname" ]; then
      hostname_found=1
      break
    fi
  done

  if [ "$hostname_found" -eq 1 ]; then
    fields[0]="$jetson_ip"
    printf "%s" "${fields[0]}"
    for ((i = 1; i < ${#fields[@]}; i++)); do
      printf " %s" "${fields[$i]}"
    done
    if [ -n "$comment" ]; then
      printf " %s" "$comment"
    fi
    printf "\n"
    mapping_updated=1
  else
    printf "%s\n" "$line"
  fi
done < /etc/hosts > "$hosts_tmp"
if [ "$mapping_updated" -eq 0 ]; then
  printf "%s %s\n" "$jetson_ip" "$jetson_hostname" >> "$hosts_tmp"
fi
cat "$hosts_tmp" > /etc/hosts
rm -f "$hosts_tmp"

legacy_start_line=""
legacy_end_line=""
line_number=0
while IFS= read -r line || [ -n "$line" ]; do
  ((line_number += 1))
  if [ "$line" = "# >>> jetson_ros1_rviz" ] && [ -z "$legacy_start_line" ]; then
    legacy_start_line="$line_number"
  elif [ "$line" = "# <<< jetson_ros1_rviz" ] && [ -n "$legacy_start_line" ]; then
    legacy_end_line="$line_number"
    break
  fi
done < "$HOME/.bashrc"

if [ -n "$legacy_start_line" ] && [ -n "$legacy_end_line" ]; then
  bashrc_tmp="$(mktemp)"
  line_number=0
  while IFS= read -r line || [ -n "$line" ]; do
    ((line_number += 1))
    if [ "$line_number" -lt "$legacy_start_line" ] || [ "$line_number" -gt "$legacy_end_line" ]; then
      printf "%s\n" "$line"
    fi
  done < "$HOME/.bashrc" > "$bashrc_tmp"
  cat "$bashrc_tmp" > "$HOME/.bashrc"
  rm -f "$bashrc_tmp"
  echo "[INFO] Removed legacy Jetson ROS settings from /root/.bashrc"
elif [ -n "$legacy_start_line" ]; then
  echo "[WARN] Found an incomplete jetson_ros1_rviz block in /root/.bashrc; left it unchanged" >&2
fi
' bash "$JETSON_IP" "$JETSON_HOSTNAME"

docker exec "${ROS_DOCKER_ENV[@]}" "$CONTAINER_NAME" bash -lc '
source /opt/ros/noetic/setup.bash
source ~/.bashrc
export ROS_MASTER_URI="$JETSON_ROS_MASTER_URI"
export ROS_IP="$JETSON_ROS_IP"
unset ROS_HOSTNAME JETSON_ROS_MASTER_URI JETSON_ROS_IP
rosnode kill /jetson_rviz_world_camera_init_tf >/dev/null 2>&1 || true
rosnode kill /jetson_rviz_world_map_tf >/dev/null 2>&1 || true
rosnode kill /rviz_goal_to_diff_planner >/dev/null 2>&1 || true
' >/dev/null 2>&1 || true

if [ "$START_GOAL_BRIDGE" = "true" ]; then
  docker exec -d "${ROS_DOCKER_ENV[@]}" "$CONTAINER_NAME" \
    bash -lc '
source /opt/ros/noetic/setup.bash
source ~/.bashrc
export ROS_MASTER_URI="$JETSON_ROS_MASTER_URI"
export ROS_IP="$JETSON_ROS_IP"
unset ROS_HOSTNAME JETSON_ROS_MASTER_URI JETSON_ROS_IP
bridge_args=("_default_z:=$2" "_input_topic:=/sim2real/rviz_goal" "_output_topic:=/goal")
if [ -n "$3" ]; then
  bridge_args+=("_frame_id:=$3")
fi
exec python3 "$1" "${bridge_args[@]}"
' bash "$GOAL_BRIDGE_CONTAINER" "$RVIZ_GOAL_Z" "$RVIZ_GOAL_FRAME"
fi

echo "[INFO] Starting RViz in container '$CONTAINER_NAME'"
echo "[INFO] ROS_MASTER_URI=$REMOTE_ROS_MASTER_URI"
echo "[INFO] ROS_IP=$LOCAL_IP"
if [ "$START_GOAL_BRIDGE" = "true" ]; then
  echo "[INFO] RViz goal bridge: /sim2real/rviz_goal -> /goal"
  echo "[INFO] 2D Nav Goal fixed height: RVIZ_GOAL_Z=$RVIZ_GOAL_Z"
else
  echo "[INFO] RViz goal bridge disabled."
fi

exec docker exec -it \
  "${ROS_DOCKER_ENV[@]}" \
  -e DISPLAY="$DISPLAY_VALUE" \
  -e QT_X11_NO_MITSHM=1 \
  "$CONTAINER_NAME" \
  bash -lc '
source /opt/ros/noetic/setup.bash
source ~/.bashrc
export ROS_MASTER_URI="$JETSON_ROS_MASTER_URI"
export ROS_IP="$JETSON_ROS_IP"
unset ROS_HOSTNAME JETSON_ROS_MASTER_URI JETSON_ROS_IP
exec rviz -d "$1"
' bash "$RVIZ_CONFIG_CONTAINER"
