#!/usr/bin/env bash
set -Eeuo pipefail

# Embedded RViz client for the localhost PX4/Gazebo simulation.  This launcher
# rejects every non-loopback ROS master so it can never fall through to a real
# aircraft while the operator believes simulation is active.
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
GROUND_STATION_CONTAINER_LAUNCHER="$PROJECT_ROOT/launch/ground_station_container.sh"
CONTAINER_NAME="${GROUND_STATION_CONTAINER:-uav_autonomy_ground_station}"
ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
ROS_IP="${ROS_IP:-127.0.0.1}"
JETSON_IP="${JETSON_IP:-127.0.0.1}"
LOCAL_IP="${LOCAL_IP:-127.0.0.1}"
ROS_MASTER_PORT="${ROS_MASTER_PORT:-11311}"
DISPLAY_VALUE="${DISPLAY:-:0}"
RVIZ_PROCESS_TOKEN="${RVIZ_PROCESS_TOKEN:-sim_rviz_${USER:-operator}_$$}"
RVIZ_CONFIG_HOST="${RVIZ_CONFIG_HOST:-$PROJECT_ROOT/deployment/config/rviz/jetson_real_stack.rviz}"
RVIZ_CONFIG_CONTAINER="/root/embedded_sim_stack.rviz"
EMBEDDED_RVIZ_HOST="$PROJECT_ROOT/deployment/ros_pkgs/sim2real_ground_station/scripts/embedded_rviz.py"
EMBEDDED_RVIZ_CONTAINER="/root/embedded_rviz.py"
INTERACTIVE_GOAL_UI_HOST="$PROJECT_ROOT/deployment/ros_pkgs/sim2real_ground_station/scripts/interactive_goal_ui.py"
INTERACTIVE_GOAL_UI_CONTAINER="/root/interactive_goal_ui.py"
RVIZ_LOCK_PATH="${RVIZ_LOCK_PATH:-$PROJECT_ROOT/runtime/tmp/ground_rviz.lock}"
RVIZ_LOCK_FD=""
XHOST_GRANTED=false

die() {
  echo "[ERROR] $*" >&2
  exit 1
}

for command_name in docker flock timeout; do
  command -v "$command_name" >/dev/null 2>&1 || die "$command_name command not found."
done

[ "$ROS_MASTER_URI" = "http://127.0.0.1:11311" ] ||
  die "Simulation RViz only accepts ROS_MASTER_URI=http://127.0.0.1:11311."
[ "$ROS_IP" = "127.0.0.1" ] || die "Simulation RViz only accepts ROS_IP=127.0.0.1."
[ "$JETSON_IP" = "127.0.0.1" ] || die "Simulation JETSON_IP must be 127.0.0.1."
[ "$LOCAL_IP" = "127.0.0.1" ] || die "Simulation LOCAL_IP must be 127.0.0.1."
[ "$ROS_MASTER_PORT" = "11311" ] || die "Simulation ROS_MASTER_PORT must be 11311."
[[ "$RVIZ_PROCESS_TOKEN" =~ ^[A-Za-z0-9_-]+$ ]] ||
  die "RVIZ_PROCESS_TOKEN may contain only letters, digits, '_' and '-'."
[ -f "$RVIZ_CONFIG_HOST" ] || die "RViz config not found: $RVIZ_CONFIG_HOST"
[ -f "$EMBEDDED_RVIZ_HOST" ] || die "Embedded RViz wrapper not found: $EMBEDDED_RVIZ_HOST"
[ -f "$INTERACTIVE_GOAL_UI_HOST" ] || die "Interactive goal UI not found: $INTERACTIVE_GOAL_UI_HOST"
[ -x "$GROUND_STATION_CONTAINER_LAUNCHER" ] || die "Ground-station container launcher not found: $GROUND_STATION_CONTAINER_LAUNCHER"

mkdir -p "$(dirname "$RVIZ_LOCK_PATH")"
exec {RVIZ_LOCK_FD}>"$RVIZ_LOCK_PATH"
flock -n "$RVIZ_LOCK_FD" || die "Another managed ground-station RViz session is active."

GROUND_STATION_CONTAINER="$CONTAINER_NAME" \
  "$GROUND_STATION_CONTAINER_LAUNCHER" verify-layout ||
  die "Ground-station image/container is stale; run ./launch/ground_station_container.sh recreate."

docker container inspect "$CONTAINER_NAME" >/dev/null 2>&1 ||
  die "Ground-station container '$CONTAINER_NAME' does not exist."
if [ "$(docker container inspect -f '{{.State.Running}}' "$CONTAINER_NAME")" != "true" ]; then
  docker start "$CONTAINER_NAME" >/dev/null
fi
[ "$(docker container inspect -f '{{.HostConfig.NetworkMode}}' "$CONTAINER_NAME")" = "host" ] ||
  die "Ground-station container must use host networking."

docker exec "$CONTAINER_NAME" bash -lc '
  source /opt/ros/noetic/setup.bash
  source ~/.bashrc
  python3 -c '\''import actionlib; from rviz import bindings as rviz; from sim2real_planning_msgs.msg import FlightCommandAction, InteractiveGoalAction; assert rviz.VisualizationFrame'\''
' >/dev/null || die "Ground-station container lacks RViz or the target-action plugin."

# Require both the guarded action and Gazebo identity.  An unrelated ROS master
# accidentally using 11311 is therefore rejected before RViz subscribes.
timeout 5 docker exec -i "$CONTAINER_NAME" python3 - "$ROS_MASTER_URI" <<'PY' ||
  die "Local AIO simulation or its guarded target action is not ready."
import socket
import sys
import xmlrpc.client

socket.setdefaulttimeout(2.0)
code, _message, state = xmlrpc.client.ServerProxy(
    sys.argv[1], allow_none=True
).getSystemState("/swarm_embedded_sim_rviz_readiness")
publishers = {str(topic) for topic, nodes in state[0] if nodes}
required = {
    "/gazebo/model_states",
    "/ground_station/interactive_goal/status",
    "/ground_station/flight_command/status",
}
raise SystemExit(0 if int(code) == 1 and required <= publishers else 1)
PY

cleanup_owned_rviz() {
  timeout 5 docker exec -e "RVIZ_PROCESS_TOKEN=$RVIZ_PROCESS_TOKEN" \
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
}

cleanup() {
  local status=$?
  trap - EXIT
  if docker container inspect "$CONTAINER_NAME" >/dev/null 2>&1; then
    cleanup_owned_rviz
  fi
  if [ "$XHOST_GRANTED" = "true" ]; then
    xhost -SI:localuser:root >/dev/null 2>&1 || true
  fi
  return "$status"
}
trap cleanup EXIT

if command -v xhost >/dev/null 2>&1 && xhost +SI:localuser:root >/dev/null 2>&1; then
  XHOST_GRANTED=true
fi

# The lock proves there is no active managed session, so wrappers left by a
# previous host crash are stale and may be removed safely.
timeout 4 docker exec -i "$CONTAINER_NAME" bash -s <<'BASH' >/dev/null 2>&1 || true
for cmdline in /proc/[0-9]*/cmdline; do
  [ -r "$cmdline" ] || continue
  arguments="$(tr '\0' ' ' < "$cmdline")"
  case "$arguments" in
    *embedded_rviz.py*--session-token*)
      pid="${cmdline#/proc/}"
      pid="${pid%/cmdline}"
      kill -TERM "$pid" >/dev/null 2>&1 || true
      ;;
  esac
done
BASH

docker cp "$RVIZ_CONFIG_HOST" "$CONTAINER_NAME:$RVIZ_CONFIG_CONTAINER"
docker cp "$EMBEDDED_RVIZ_HOST" "$CONTAINER_NAME:$EMBEDDED_RVIZ_CONTAINER"
docker cp "$INTERACTIVE_GOAL_UI_HOST" "$CONTAINER_NAME:$INTERACTIVE_GOAL_UI_CONTAINER"
docker exec "$CONTAINER_NAME" chmod 755 \
  "$EMBEDDED_RVIZ_CONTAINER" "$INTERACTIVE_GOAL_UI_CONTAINER"

echo "[INFO] Starting embedded RViz for localhost AIO simulation"
echo "[INFO] ROS_MASTER_URI=$ROS_MASTER_URI"
docker exec -i \
  -e "DISPLAY=$DISPLAY_VALUE" \
  -e QT_X11_NO_MITSHM=1 \
  -e SWARM_RVIZ_SIMULATION=1 \
  -e "ROS_MASTER_URI=$ROS_MASTER_URI" \
  -e "ROS_IP=$ROS_IP" \
  "$CONTAINER_NAME" bash -lc '
source /opt/ros/noetic/setup.bash
source ~/.bashrc
export ROS_MASTER_URI="$1" ROS_IP="$2"
unset ROS_HOSTNAME
exec python3 "$3" "$4" --title "Local Simulation UAV" --session-token "$5"
' bash "$ROS_MASTER_URI" "$ROS_IP" "$EMBEDDED_RVIZ_CONTAINER" \
  "$RVIZ_CONFIG_CONTAINER" "$RVIZ_PROCESS_TOKEN"
