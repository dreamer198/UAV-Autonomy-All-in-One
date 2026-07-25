#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONTAINER_NAME="${CONTAINER_NAME:-diff_planner_px4_real}"
RUNTIME_DIR="${RUNTIME_DIR:-$PROJECT_ROOT/runtime}"
SESSION_NAME="${BAG_SESSION_NAME:-real_bag_playback}"
REAL_SESSION_NAME="${REAL_SESSION_NAME:-real_px4_stack}"
BAG_RATE="${BAG_RATE:-1.0}"
BAG_LOOP="${BAG_LOOP:-false}"
ROS_IP="${ROS_IP:-}"
CONVERTER_HOST="$PROJECT_ROOT/deployment/ros_pkgs/sim2real_deployment/scripts/livox_custom_to_pointcloud2.py"
CONVERTER_CONTAINER="/root/tmp/livox_custom_to_pointcloud2.py"
MASTER_MARKER="$RUNTIME_DIR/.real_bag_roscore_owned"

usage() {
  cat <<'EOF'
Usage: real_bag.sh {play [bag-file]|stop|status|attach}

Environment:
  BAG_RATE=1.0       Playback speed.
  BAG_LOOP=false     Set true to loop until stopped.
  RUNTIME_DIR=...    Must match the directory used to create the real container.
  ROS_IP=...         Jetson address advertised to a remote RViz.
EOF
}

need_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "[ERROR] Missing command: $1" >&2
    exit 1
  fi
}

container_running() {
  [ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER_NAME" 2>/dev/null || true)" = "true" ]
}

detect_ros_ip() {
  hostname -I | tr ' ' '\n' | awk '/^10\.0\.30\./ {print; exit}'
}

docker_ros() {
  docker exec "$CONTAINER_NAME" bash -lc \
    "export ROS_MASTER_URI=http://127.0.0.1:11311; source /opt/ros/noetic/setup.bash; source /root/catkin_ws/devel/setup.bash; $1"
}

master_running() {
  container_running && docker_ros 'rosparam get /run_id >/dev/null 2>&1'
}

flight_nodes_running() {
  docker_ros "rosnode list 2>/dev/null | grep -Eq '^/(mavros|se3_controller|planning_node|traj_server|fast_lio|livox_lidar_publisher)(/|$)'"
}

stop_playback() {
  if container_running; then
    docker exec "$CONTAINER_NAME" bash -lc '
      for pattern in "[r]osbag play" "[l]ivox_custom_to_pointcloud2.py" "[s]tatic_transform_publisher.*offline_body_to_livox_tf"; do
        while read -r pid; do
          [ -n "$pid" ] && kill -INT "$pid" 2>/dev/null || true
        done < <(pgrep -f "$pattern" || true)
      done
    ' || true
  fi
  if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
    tmux kill-session -t "$SESSION_NAME"
  fi
  if [ -f "$MASTER_MARKER" ] && ! tmux has-session -t "$REAL_SESSION_NAME" 2>/dev/null; then
    if container_running; then
      docker exec "$CONTAINER_NAME" bash -lc '
        for pattern in "/opt/ros/noetic/bin/[r]oscore" "/opt/ros/noetic/bin/[r]osmaster"; do
          while read -r pid; do
            [ -n "$pid" ] && kill -INT "$pid" 2>/dev/null || true
          done < <(pgrep -f "$pattern" || true)
        done
      ' || true
    fi
    rm -f "$MASTER_MARKER"
  fi
}

latest_bag() {
  find "$RUNTIME_DIR/flight_bags" -maxdepth 1 -type f -name '*.bag' \
    -printf '%T@ %p\n' | sort -n | tail -n 1 | cut -d' ' -f2-
}

resolve_bag() {
  local requested="${1:-}"
  local host_path
  if [ -z "$requested" ]; then
    host_path="$(latest_bag)"
  elif [[ "$requested" = /* ]]; then
    host_path="$requested"
  else
    host_path="$PROJECT_ROOT/$requested"
  fi

  if [ -z "$host_path" ] || [ ! -f "$host_path" ]; then
    echo "[ERROR] Bag file not found: ${host_path:-<none>}" >&2
    return 1
  fi

  host_path="$(readlink -f "$host_path")"
  local bag_root
  bag_root="$(readlink -f "$RUNTIME_DIR/flight_bags")"
  case "$host_path" in
    "$bag_root"/*) printf '/root/flight_bags/%s\n' "${host_path#"$bag_root"/}" ;;
    *)
      echo "[ERROR] Bag must be inside $bag_root so the container can read it." >&2
      return 1
      ;;
  esac
}

play_bag() {
  local requested="${1:-}"
  local bag_container loop_arg="" master_owned=false

  if tmux has-session -t "$REAL_SESSION_NAME" 2>/dev/null; then
    echo "[ERROR] Real-flight stack '$REAL_SESSION_NAME' is running. Stop it before offline playback." >&2
    return 1
  fi
  if ! container_running; then
    echo "[ERROR] Container '$CONTAINER_NAME' is not running." >&2
    return 1
  fi
  if [ ! -f "$CONVERTER_HOST" ]; then
    echo "[ERROR] Converter not found: $CONVERTER_HOST" >&2
    return 1
  fi
  if ! [[ "$BAG_RATE" =~ ^[0-9]+([.][0-9]+)?$ ]] || [ "$BAG_RATE" = "0" ] || [ "$BAG_RATE" = "0.0" ]; then
    echo "[ERROR] BAG_RATE must be positive." >&2
    return 1
  fi
  if [ "$BAG_LOOP" != "true" ] && [ "$BAG_LOOP" != "false" ]; then
    echo "[ERROR] BAG_LOOP must be true or false." >&2
    return 1
  fi

  bag_container="$(resolve_bag "$requested")"
  [ "$BAG_LOOP" = "true" ] && loop_arg="--loop"
  [ -n "$ROS_IP" ] || ROS_IP="$(detect_ros_ip)"
  if [ -z "$ROS_IP" ]; then
    echo "[ERROR] Unable to detect Jetson ROS_IP; set ROS_IP explicitly." >&2
    return 1
  fi

  stop_playback
  docker cp "$CONVERTER_HOST" "$CONTAINER_NAME:$CONVERTER_CONTAINER"
  docker exec "$CONTAINER_NAME" chmod +x "$CONVERTER_CONTAINER"

  if master_running; then
    if flight_nodes_running; then
      echo "[ERROR] A ROS master with real-flight nodes is already running. Stop the flight stack before playback." >&2
      return 1
    fi
    tmux new-session -d -s "$SESSION_NAME" -n external_master \
      "while sleep 3600; do :; done"
    echo "[INFO] Reusing the existing ROS master (it will not be stopped by this script)."
  else
    tmux new-session -d -s "$SESSION_NAME" -n roscore \
      "docker exec $CONTAINER_NAME bash -lc 'export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=$ROS_IP; source /opt/ros/noetic/setup.bash; source /root/catkin_ws/devel/setup.bash; exec roscore'"
    master_owned=true
  fi

  local ready=false
  for _ in $(seq 1 30); do
    if docker_ros 'rosparam get /run_id >/dev/null 2>&1'; then
      ready=true
      break
    fi
    sleep 0.5
  done
  if [ "$ready" != "true" ]; then
    echo "[ERROR] ROS master did not become ready." >&2
    stop_playback
    return 1
  fi
  if [ "$master_owned" = "true" ]; then
    mkdir -p "$(dirname "$MASTER_MARKER")"
    printf '%s\n' "$SESSION_NAME" > "$MASTER_MARKER"
  fi
  docker_ros 'rosparam set /use_sim_time true'

  tmux new-window -t "$SESSION_NAME" -n raw_converter \
    "docker exec $CONTAINER_NAME bash -lc 'export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=$ROS_IP; source /opt/ros/noetic/setup.bash; source /root/livox_ws/devel/setup.bash; source /root/catkin_ws/devel/setup.bash; exec python3 $CONVERTER_CONTAINER _input_topic:=/livox/lidar _output_topic:=/livox/lidar_points'"
  tmux new-window -t "$SESSION_NAME" -n lidar_tf \
    "docker exec $CONTAINER_NAME bash -lc 'export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=$ROS_IP; source /opt/ros/noetic/setup.bash; exec rosrun tf2_ros static_transform_publisher -0.011 -0.02329 0.04412 0 0 0 body livox_frame __name:=offline_body_to_livox_tf'"

  for _ in $(seq 1 20); do
    if docker_ros 'rosnode list | grep -qx /livox_custom_to_pointcloud2'; then
      break
    fi
    sleep 0.25
  done

  tmux new-window -t "$SESSION_NAME" -n rosbag \
    "docker exec $CONTAINER_NAME bash -lc 'export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=$ROS_IP; source /opt/ros/noetic/setup.bash; source /root/livox_ws/devel/setup.bash; source /root/catkin_ws/devel/setup.bash; exec rosbag play --clock --rate=$BAG_RATE $loop_arg $bag_container'"

  echo "[INFO] Offline bag playback started."
  echo "[INFO] Bag: $bag_container"
  echo "[INFO] Rate: $BAG_RATE, loop: $BAG_LOOP"
  echo "[INFO] Raw-density display: /livox/lidar -> /livox/lidar_points"
  echo "[INFO] Session: $SESSION_NAME"
}

show_status() {
  if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
    tmux list-windows -t "$SESSION_NAME"
    if container_running; then
      docker_ros 'rosnode list' || true
    fi
  else
    echo "[INFO] Offline playback is not running."
  fi
}

main() {
  need_cmd docker
  need_cmd tmux
  case "${1:-}" in
    play) play_bag "${2:-}" ;;
    stop) stop_playback ;;
    status) show_status ;;
    attach)
      if ! tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
        echo "[ERROR] Offline playback is not running." >&2
        exit 1
      fi
      exec tmux attach -t "$SESSION_NAME"
      ;;
    *) usage; exit 1 ;;
  esac
}

main "$@"
