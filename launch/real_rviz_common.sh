# shellcheck shell=bash
# Shared implementation for the dedicated live and offline RViz entrypoints.
# The entrypoint assigns REAL_RVIZ_ENTRYPOINT_KIND unconditionally before
# sourcing this file; it is intentionally not configurable through the
# environment.

case "${REAL_RVIZ_ENTRYPOINT_KIND:-}" in
  live|offline_bag) ;;
  *)
    echo "[ERROR] real_rviz_common.sh must be sourced by a supported RViz entrypoint." >&2
    return 1 2>/dev/null || exit 1
    ;;
esac

CONTAINER_NAME="${GROUND_STATION_CONTAINER:-${CONTAINER_NAME:-uav_autonomy_ground_station}}"
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
GROUND_STATION_CONTAINER_LAUNCHER="$PROJECT_ROOT/launch/ground_station_container.sh"
JETSON_IP="${JETSON_IP:-192.168.1.123}"
LOCAL_IP="${LOCAL_IP:-192.168.1.124}"
ROS_MASTER_PORT="${ROS_MASTER_PORT:-11312}"
JETSON_HOSTNAME="${JETSON_HOSTNAME:-jetson2-desktop}"
RVIZ_CONFIG_HOST="${RVIZ_CONFIG_HOST:-$PROJECT_ROOT/deployment/config/rviz/jetson_real_stack.rviz}"
RVIZ_CONFIG_CONTAINER="${RVIZ_CONFIG_CONTAINER:-/root/jetson_real_stack.rviz}"
EMBEDDED_RVIZ_HOST="${EMBEDDED_RVIZ_HOST:-$PROJECT_ROOT/deployment/ros_pkgs/sim2real_ground_station/scripts/embedded_rviz.py}"
EMBEDDED_RVIZ_CONTAINER="${EMBEDDED_RVIZ_CONTAINER:-/root/embedded_rviz.py}"
INTERACTIVE_GOAL_UI_HOST="${INTERACTIVE_GOAL_UI_HOST:-$PROJECT_ROOT/deployment/ros_pkgs/sim2real_ground_station/scripts/interactive_goal_ui.py}"
INTERACTIVE_GOAL_UI_CONTAINER="${INTERACTIVE_GOAL_UI_CONTAINER:-/root/interactive_goal_ui.py}"
VISUALIZATION_SCRIPTS_HOST="${VISUALIZATION_SCRIPTS_HOST:-$PROJECT_ROOT/simulation/ros_pkgs/sim2real_simulation/scripts}"
ENVIRONMENT_MAP_HOST="${ENVIRONMENT_MAP_HOST:-$VISUALIZATION_SCRIPTS_HOST/environment_voxel_map.py}"
ENVIRONMENT_VIZ_HOST="${ENVIRONMENT_VIZ_HOST:-$VISUALIZATION_SCRIPTS_HOST/stable_environment_viz.py}"
FLIGHT_PATH_HOST="${FLIGHT_PATH_HOST:-$VISUALIZATION_SCRIPTS_HOST/flight_path_history.py}"
FLIGHT_VIZ_HOST="${FLIGHT_VIZ_HOST:-$VISUALIZATION_SCRIPTS_HOST/flight_visualization.py}"
ENVIRONMENT_MAP_CONTAINER="${ENVIRONMENT_MAP_CONTAINER:-/root/environment_voxel_map.py}"
ENVIRONMENT_VIZ_CONTAINER="${ENVIRONMENT_VIZ_CONTAINER:-/root/stable_environment_viz.py}"
FLIGHT_PATH_CONTAINER="${FLIGHT_PATH_CONTAINER:-/root/flight_path_history.py}"
FLIGHT_VIZ_CONTAINER="${FLIGHT_VIZ_CONTAINER:-/root/flight_visualization.py}"
DISPLAY_VALUE="${DISPLAY:-:0}"
RVIZ_SESSION_KIND="$REAL_RVIZ_ENTRYPOINT_KIND"
REAL_RVIZ_EMBEDDED="${REAL_RVIZ_EMBEDDED:-false}"
RVIZ_PROCESS_TOKEN="${RVIZ_PROCESS_TOKEN:-rviz_${USER:-operator}_$$}"
RVIZ_LOCK_PATH="${RVIZ_LOCK_PATH:-$PROJECT_ROOT/runtime/tmp/ground_rviz.lock}"
RVIZ_HELPER_START_DELAY="${RVIZ_HELPER_START_DELAY:-15}"
RVIZ_LOCK_FD=""
XHOST_GRANTED=false

is_live_session() {
  [ "$RVIZ_SESSION_KIND" = "live" ]
}

master_has_publishers() {
  local master_uri="$1"
  shift
  # Query only the ROS master.  rosnode/rostopic CLI calls may block while
  # resolving every remote node URI on a loaded Jetson, which must never hold
  # up creation of the embedded RViz window.
  timeout 5 docker exec -i "$CONTAINER_NAME" \
    python3 - "$master_uri" "$@" <<'PY'
import socket
import sys
import xmlrpc.client

socket.setdefaulttimeout(2.0)
master_uri = sys.argv[1]
required_topics = set(sys.argv[2:])
try:
    code, _message, state = xmlrpc.client.ServerProxy(
        master_uri, allow_none=True
    ).getSystemState("/ground_rviz_readiness")
    publishers = {
        str(topic)
        for topic, nodes in state[0]
        if nodes
    }
except Exception:
    raise SystemExit(1)
raise SystemExit(0 if int(code) == 1 and required_topics <= publishers else 1)
PY
}

stop_ground_rviz_helpers() {
  local process_token="${1:-}"
  # Stop only processes owned by this ground-station integration.  Inspecting
  # local /proc is deterministic even when the remote ROS master is loaded;
  # rosnode kill can block while resolving unrelated Jetson node XML-RPC URIs.
  timeout 3 docker exec -i "$CONTAINER_NAME" bash -s -- "$process_token" <<'BASH' >/dev/null 2>&1 || true
process_token="$1"
for cmdline in /proc/[0-9]*/cmdline; do
  [ -r "$cmdline" ] || continue
  arguments="$(tr '\0' ' ' < "$cmdline")"
  owned=false
  if [ -n "$process_token" ]; then
    # Match both a helper after exec (the token is a private ROS parameter)
    # and its delayed bash wrapper (the token is its final argv entry).
    case "$arguments" in
      *stable_environment_viz.py*"$process_token"*|\
      *flight_visualization.py*"$process_token"*) owned=true ;;
    esac
  else
    # Startup orphan cleanup.  The flock above guarantees there is no other
    # managed RViz session, so these integration-specific ROS names are stale.
    case "$arguments" in
      *stable_environment_viz.py*__name:=ground_rviz_environment*|\
      *flight_visualization.py*__name:=ground_rviz_flight_visualization*|\
      *static_transform_publisher*__name:=jetson_rviz_world_camera_init_tf*|\
      *static_transform_publisher*__name:=jetson_rviz_world_map_tf*) owned=true ;;
    esac
  fi
  if [ "$owned" = "true" ]; then
    pid="${cmdline#/proc/}"
    pid="${pid%/cmdline}"
    [ "$pid" = "$$" ] || kill -KILL "$pid" >/dev/null 2>&1 || true
  fi
done
BASH
}

stop_stale_embedded_rviz() {
  # A previous host crash can leave an RViz node subscribed to every large
  # cloud topic.  Remove only our embedded wrapper processes; never match a
  # standalone operator RViz session.
  timeout 4 docker exec -i "$CONTAINER_NAME" bash -s <<'BASH' >/dev/null 2>&1 || true
for cmdline in /proc/[0-9]*/cmdline; do
  [ -r "$cmdline" ] || continue
  arguments="$(tr '\0' ' ' < "$cmdline")"
  case "$arguments" in
    *embedded_rviz.py*--session-token*)
      pid="${cmdline#/proc/}"
      pid="${pid%/cmdline}"
      [ "$pid" = "$$" ] || kill -TERM "$pid" >/dev/null 2>&1 || true
      ;;
  esac
done
BASH
}

for command_name in docker flock ip timeout; do
  if ! command -v "$command_name" >/dev/null 2>&1; then
    echo "[ERROR] $command_name command not found."
    exit 1
  fi
done
[ -x "$GROUND_STATION_CONTAINER_LAUNCHER" ] || {
  echo "[ERROR] Ground-station container launcher is missing: $GROUND_STATION_CONTAINER_LAUNCHER" >&2
  exit 1
}
if is_live_session && ! command -v python3 >/dev/null 2>&1; then
  echo "[ERROR] python3 command not found."
  exit 1
fi

detect_local_ip() {
  local detected=""
  detected="$(ip -4 route get "$JETSON_IP" 2>/dev/null |
    awk '{for (i=1; i<=NF; i++) if ($i == "src") {print $(i+1); exit}}')" || true
  if [ -z "$detected" ]; then
    detected="$(hostname -I 2>/dev/null | awk '{print $1}')" || true
  fi
  printf '%s\n' "$detected"
}

if [ -z "$LOCAL_IP" ]; then
  echo "[ERROR] Could not detect the workstation address on the route to $JETSON_IP. Set LOCAL_IP manually."
  exit 1
fi

case "$REAL_RVIZ_EMBEDDED" in
  true|false) ;;
  *)
    echo "[ERROR] REAL_RVIZ_EMBEDDED must be exactly 'true' or 'false'." >&2
    exit 1
    ;;
esac
if ! [[ "$RVIZ_PROCESS_TOKEN" =~ ^[A-Za-z0-9_-]+$ ]]; then
  echo "[ERROR] RVIZ_PROCESS_TOKEN may contain only letters, digits, '_' and '-'." >&2
  exit 1
fi
if ! [[ "$RVIZ_HELPER_START_DELAY" =~ ^([0-9]|[1-5][0-9]|60)$ ]]; then
  echo "[ERROR] RVIZ_HELPER_START_DELAY must be an integer from 0 to 60 seconds." >&2
  exit 1
fi

mkdir -p "$(dirname "$RVIZ_LOCK_PATH")"
exec {RVIZ_LOCK_FD}>"$RVIZ_LOCK_PATH"
if ! flock -n "$RVIZ_LOCK_FD"; then
  echo "[ERROR] Another ground-station RViz session already owns $RVIZ_LOCK_PATH." >&2
  exit 1
fi

if ! [[ "$ROS_MASTER_PORT" =~ ^[0-9]+$ ]] ||
  [ "$ROS_MASTER_PORT" -lt 1 ] || [ "$ROS_MASTER_PORT" -gt 65535 ]; then
  echo "[ERROR] ROS_MASTER_PORT must be an integer from 1 to 65535." >&2
  exit 1
fi
REMOTE_ROS_MASTER_URI="http://$JETSON_IP:$ROS_MASTER_PORT"
ROS_DOCKER_ENV=(
  -e "JETSON_ROS_MASTER_URI=$REMOTE_ROS_MASTER_URI"
  -e "JETSON_ROS_IP=$LOCAL_IP"
)

# The wrapper and Python UI are copied in at launch, but generated Action
# classes live in the image.  Reject a stale schema/image before starting a
# container that could otherwise connect to the aircraft with mismatched MD5s.
if ! GROUND_STATION_CONTAINER="$CONTAINER_NAME" \
  "$GROUND_STATION_CONTAINER_LAUNCHER" verify-layout; then
  echo "[ERROR] Ground-station image/container is stale. Recreate it with: $GROUND_STATION_CONTAINER_LAUNCHER recreate" >&2
  exit 1
fi

if ! docker container inspect "$CONTAINER_NAME" >/dev/null 2>&1; then
  echo "[ERROR] Docker container '$CONTAINER_NAME' does not exist."
  echo "[ERROR] Create it on the ground station with: ./launch/ground_station_container.sh run"
  exit 1
fi

if [ "$(docker container inspect -f '{{.State.Running}}' "$CONTAINER_NAME")" != "true" ]; then
  echo "[INFO] Starting container: $CONTAINER_NAME"
  docker start "$CONTAINER_NAME" >/dev/null
fi

if [ "$(docker container inspect -f '{{.HostConfig.NetworkMode}}' "$CONTAINER_NAME")" != "host" ]; then
  echo "[ERROR] Ground-station container '$CONTAINER_NAME' must use Docker host networking for bidirectional ROS 1 traffic."
  exit 1
fi

if ! docker exec "$CONTAINER_NAME" bash -lc '
  source /opt/ros/noetic/setup.bash
  source ~/.bashrc
  command -v rviz >/dev/null
'; then
  echo "[ERROR] Container '$CONTAINER_NAME' does not provide ROS Noetic and RViz."
  echo "[ERROR] Rebuild it with: ./launch/ground_station_container.sh build"
  exit 1
fi

if [ ! -f "$RVIZ_CONFIG_HOST" ]; then
  echo "[ERROR] RViz config not found: $RVIZ_CONFIG_HOST"
  exit 1
fi

if is_live_session; then
  if ! docker exec "$CONTAINER_NAME" bash -lc '
    source /opt/ros/noetic/setup.bash
    source ~/.bashrc
    command -v rosservice >/dev/null
    python3 -c '\''import actionlib, numpy, tf2_ros; from rviz import bindings as rviz; assert rviz.VisualizationFrame; from mavros_msgs.msg import State; from nav_msgs.msg import Odometry; from sensor_msgs import point_cloud2; from sensor_msgs.msg import PointCloud2; from sim2real_planning_msgs.msg import FlightCommandAction, InteractiveGoalAction, PlannerGoal, PlannerStatus; from sim2real_planning_msgs.srv import ValidateGoal; from std_srvs.srv import Trigger; from visualization_msgs.msg import Marker'\''
  '; then
    echo "[ERROR] Container '$CONTAINER_NAME' lacks the ROS dependencies required by live RViz visualization and goal forwarding."
    echo "[ERROR] Rebuild it with: ./launch/ground_station_container.sh build"
    exit 1
  fi

  for visualization_source in \
    "$ENVIRONMENT_MAP_HOST" \
    "$ENVIRONMENT_VIZ_HOST" \
    "$FLIGHT_PATH_HOST" \
    "$FLIGHT_VIZ_HOST"; do
    if [ ! -f "$visualization_source" ]; then
      echo "[ERROR] Ground-station visualization source not found: $visualization_source"
      exit 1
    fi
  done

fi

cleanup() {
  local status=$?
  trap - EXIT
  if is_live_session &&
    docker container inspect "$CONTAINER_NAME" >/dev/null 2>&1 &&
    [ "$(docker container inspect -f '{{.State.Running}}' "$CONTAINER_NAME" 2>/dev/null || true)" = "true" ]; then
    stop_ground_rviz_helpers "$RVIZ_PROCESS_TOKEN"
  fi
  if [ "$REAL_RVIZ_EMBEDDED" = "true" ] &&
    docker container inspect "$CONTAINER_NAME" >/dev/null 2>&1 &&
    [ "$(docker container inspect -f '{{.State.Running}}' "$CONTAINER_NAME" 2>/dev/null || true)" = "true" ]; then
    timeout 5 docker exec \
      -e "RVIZ_PROCESS_TOKEN=$RVIZ_PROCESS_TOKEN" \
      "$CONTAINER_NAME" bash -lc '
      for cmdline in /proc/[0-9]*/cmdline; do
        [ -r "$cmdline" ] || continue
        arguments="$(tr "\0" " " < "$cmdline")"
        case "$arguments" in
          *embedded_rviz.py*"--session-token $RVIZ_PROCESS_TOKEN"*)
            pid="${cmdline#/proc/}"
            pid="${pid%/cmdline}"
            kill -TERM "$pid" >/dev/null 2>&1 || true
            ;;
        esac
      done
    ' >/dev/null 2>&1 || true
  fi
  if [ "$XHOST_GRANTED" = "true" ]; then
    xhost -SI:localuser:root >/dev/null 2>&1 || true
  fi
  return "$status"
}
trap cleanup EXIT

if command -v xhost >/dev/null 2>&1; then
  if xhost +SI:localuser:root >/dev/null 2>&1; then
    XHOST_GRANTED=true
  fi
fi

if [ "$REAL_RVIZ_EMBEDDED" = "true" ]; then
  if [ ! -f "$EMBEDDED_RVIZ_HOST" ]; then
    echo "[ERROR] Embedded RViz wrapper not found: $EMBEDDED_RVIZ_HOST"
    exit 1
  fi
  if [ ! -f "$INTERACTIVE_GOAL_UI_HOST" ]; then
    echo "[ERROR] Interactive goal UI not found: $INTERACTIVE_GOAL_UI_HOST"
    exit 1
  fi
  stop_stale_embedded_rviz
fi

docker cp "$RVIZ_CONFIG_HOST" "$CONTAINER_NAME:$RVIZ_CONFIG_CONTAINER"
if [ "$REAL_RVIZ_EMBEDDED" = "true" ]; then
  docker cp "$EMBEDDED_RVIZ_HOST" "$CONTAINER_NAME:$EMBEDDED_RVIZ_CONTAINER"
  docker cp "$INTERACTIVE_GOAL_UI_HOST" "$CONTAINER_NAME:$INTERACTIVE_GOAL_UI_CONTAINER"
  docker exec "$CONTAINER_NAME" chmod 755 \
    "$EMBEDDED_RVIZ_CONTAINER" "$INTERACTIVE_GOAL_UI_CONTAINER"
fi
if is_live_session; then
  docker cp "$ENVIRONMENT_MAP_HOST" "$CONTAINER_NAME:$ENVIRONMENT_MAP_CONTAINER"
  docker cp "$ENVIRONMENT_VIZ_HOST" "$CONTAINER_NAME:$ENVIRONMENT_VIZ_CONTAINER"
  docker cp "$FLIGHT_PATH_HOST" "$CONTAINER_NAME:$FLIGHT_PATH_CONTAINER"
  docker cp "$FLIGHT_VIZ_HOST" "$CONTAINER_NAME:$FLIGHT_VIZ_CONTAINER"
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

if is_live_session; then
  stop_ground_rviz_helpers

  # The UI manager already probes this action before launching RViz.  Keep one
  # fail-closed check at the launcher boundary before any visualization
  # subscriber is allowed to pull point-cloud data from the Jetson.
  action_ready=false
  if master_has_publishers \
    "$REMOTE_ROS_MASTER_URI" \
    /ground_station/interactive_goal/status \
    /ground_station/flight_command/status \
    >/dev/null 2>&1; then
    action_ready=true
  fi
  if [ "$action_ready" != "true" ]; then
    echo "[ERROR] Jetson ground-station actions are unavailable: /ground_station/interactive_goal and /ground_station/flight_command." >&2
    exit 1
  fi

  # Generate the simulation-equivalent human-facing layers on the ground
  # station. They only publish RViz topics; planner and flight-control inputs
  # remain owned by the Jetson stack.  Both helpers intentionally wait until
  # after the librviz window normally exists, so first paint always has
  # priority over point-cloud transport and voxel-map construction.
  docker exec -d "${ROS_DOCKER_ENV[@]}" \
    -e "RVIZ_HELPER_START_DELAY=$RVIZ_HELPER_START_DELAY" \
    -e "RVIZ_PROCESS_TOKEN=$RVIZ_PROCESS_TOKEN" \
    "$CONTAINER_NAME" \
    bash -lc '
sleep "$RVIZ_HELPER_START_DELAY"
source /opt/ros/noetic/setup.bash
source ~/.bashrc
export ROS_MASTER_URI="$JETSON_ROS_MASTER_URI"
export ROS_IP="$JETSON_ROS_IP"
helper_token="$2"
unset ROS_HOSTNAME JETSON_ROS_MASTER_URI JETSON_ROS_IP RVIZ_HELPER_START_DELAY RVIZ_PROCESS_TOKEN
exec python3 "$1" \
  __name:=ground_rviz_environment \
  _ground_rviz_session_token:="$helper_token" \
  _input_topic:=/ground_station/cloud_registered \
  _output_topic:=/planning/viz/environment \
  _world_frame:=world \
  _sensor_frame:=base_link \
  _publish_rate:=1.0
' bash "$ENVIRONMENT_VIZ_CONTAINER" "$RVIZ_PROCESS_TOKEN"

  docker exec -d "${ROS_DOCKER_ENV[@]}" \
    -e "RVIZ_HELPER_START_DELAY=$RVIZ_HELPER_START_DELAY" \
    -e "RVIZ_PROCESS_TOKEN=$RVIZ_PROCESS_TOKEN" \
    "$CONTAINER_NAME" \
    bash -lc '
sleep "$RVIZ_HELPER_START_DELAY"
source /opt/ros/noetic/setup.bash
source ~/.bashrc
export ROS_MASTER_URI="$JETSON_ROS_MASTER_URI"
export ROS_IP="$JETSON_ROS_IP"
helper_token="$2"
unset ROS_HOSTNAME JETSON_ROS_MASTER_URI JETSON_ROS_IP RVIZ_HELPER_START_DELAY RVIZ_PROCESS_TOKEN
exec python3 "$1" \
  __name:=ground_rviz_flight_visualization \
  _ground_rviz_session_token:="$helper_token" \
  _world_frame:=world \
  _odom_topic:=/localization/odom \
  _state_topic:=/mavros/state \
  _status_topic:=/planning/status \
  _path_topic:=/planning/viz/executed_path \
  _active_goal_marker_topic:=/planning/viz/active_goal
' bash "$FLIGHT_VIZ_CONTAINER" "$RVIZ_PROCESS_TOKEN"
fi

echo "[INFO] Starting RViz in container '$CONTAINER_NAME'"
echo "[INFO] ROS_MASTER_URI=$REMOTE_ROS_MASTER_URI"
echo "[INFO] ROS_IP=$LOCAL_IP"
if is_live_session; then
  echo "[INFO] Ground visualization: persistent environment, active goal, and measured flight path"
  echo "[INFO] Guarded target action: /ground_station/interactive_goal"
  echo "[INFO] Guarded Takeoff/Land action: /ground_station/flight_command"
else
  echo "[INFO] Starting the dedicated offline-bag RViz session."
fi

if [ "$REAL_RVIZ_EMBEDDED" = "true" ]; then
  docker exec -i \
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
exec python3 "$3" \
  "$1" --session-token "$2"
' bash "$RVIZ_CONFIG_CONTAINER" "$RVIZ_PROCESS_TOKEN" "$EMBEDDED_RVIZ_CONTAINER"
else
  docker exec -it \
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
fi
