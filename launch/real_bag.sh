#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONTAINER_NAME="${CONTAINER_NAME:-uav_autonomy_real}"
RUNTIME_DIR="${RUNTIME_DIR:-$PROJECT_ROOT/runtime}"
SESSION_NAME="${BAG_SESSION_NAME:-real_bag_playback}"
REAL_SESSION_NAME="${REAL_SESSION_NAME:-real_px4_stack}"
BAG_RATE="${BAG_RATE:-1.0}"
BAG_LOOP="${BAG_LOOP:-false}"
ROS_IP="${ROS_IP:-}"
ROS_IP_TARGET="${ROS_IP_TARGET:-1.1.1.1}"
CONVERTER_HOST="$PROJECT_ROOT/deployment/ros_pkgs/sim2real_deployment/scripts/livox_custom_to_pointcloud2.py"
CONVERTER_CONTAINER="/root/tmp/livox_custom_to_pointcloud2.py"
MASTER_MARKER="$RUNTIME_DIR/.real_bag_roscore_owned"
PLAYBACK_MARKER="$RUNTIME_DIR/.real_bag_playback_owned"
PLAYBACK_TOKEN_OPTION="@uav_autonomy_playback_token"

usage() {
  cat <<'EOF'
Usage: real_bag.sh {play [bag-file]|stop|status|attach}

Environment:
  BAG_RATE=1.0       Strictly positive playback speed.
  BAG_LOOP=false     Set true to loop until stopped.
  RUNTIME_DIR=...    Must match the directory used to create the real container.
  ROS_IP=...         Optional address advertised to remote ROS clients.
  ROS_IP_TARGET=...  Route target used for automatic ROS_IP detection.
EOF
}

need_cmd() {
  command -v "$1" >/dev/null 2>&1 || {
    echo "[ERROR] Missing command: $1" >&2
    exit 1
  }
}

container_running() {
  [ "$(docker container inspect -f '{{.State.Running}}' "$CONTAINER_NAME" 2>/dev/null || true)" = "true" ]
}

detect_ros_ip() {
  local detected=""
  detected="$(ip -4 route get "$ROS_IP_TARGET" 2>/dev/null |
    awk '{for (i=1; i<=NF; i++) if ($i == "src") {print $(i+1); exit}}')" || true
  if [ -z "$detected" ]; then
    detected="$(hostname -I 2>/dev/null | awk '{print $1}')" || true
  fi
  printf '%s\n' "$detected"
}

docker_ros() {
  docker exec "$CONTAINER_NAME" bash -lc \
    "export ROS_MASTER_URI=http://127.0.0.1:11312; source /opt/ros/noetic/setup.bash; source /root/catkin_ws/devel/setup.bash; $1"
}

master_running() {
  container_running && docker_ros 'rosparam get /run_id >/dev/null 2>&1'
}

flight_nodes_running() {
  docker_ros "rosnode list 2>/dev/null | grep -Eq '^/(mavros(/|$)|planning/backends/super/(planner|adapter)$|se3_controller_node$|interactive_goal_server$|flight_command_server$|drone_0_|fastlio_mapping$|livox_lidar_publisher|localization_guard$|odom_to_base$|odom_to_pose$|cloud_relay$|trajectory_msg_converter$|flight_recorder$)'"
}

flight_processes_running() {
  container_running || return 1
  docker top "$CONTAINER_NAME" -eo args 2>/dev/null |
    grep -Eq 'mavros_node|se3_controller_node|interactive_goal_server\.py|flight_command_server\.py|diff_planner_node|super_backend_adapter_node|super_planner/fsm_node|traj_server|fastlio_mapping|livox_ros_driver2_node|localization_guard\.py|trajectory_msg_converter\.py|/rosbag[[:space:]]+record|/rosbag/record[[:space:]]'
}

write_master_marker() {
  local token="$1" marker_tmp="${MASTER_MARKER}.tmp.$$"
  mkdir -p "$(dirname "$MASTER_MARKER")"
  {
    printf 'container=%s\n' "$CONTAINER_NAME"
    printf 'session=%s\n' "$SESSION_NAME"
    printf 'token=%s\n' "$token"
  } > "$marker_tmp"
  mv -f -- "$marker_tmp" "$MASTER_MARKER"
}

claim_playback_session() {
  local token="$1" marker_tmp="${PLAYBACK_MARKER}.tmp.$$"
  tmux set-option -t "$SESSION_NAME" "$PLAYBACK_TOKEN_OPTION" "$token" ||
    return 1
  mkdir -p "$(dirname "$PLAYBACK_MARKER")"
  {
    printf 'session=%s\n' "$SESSION_NAME"
    printf 'token=%s\n' "$token"
  } > "$marker_tmp"
  mv -f -- "$marker_tmp" "$PLAYBACK_MARKER"
}

playback_marker_valid() {
  [ -f "$PLAYBACK_MARKER" ] || return 1
  local marker_session="" marker_token="" key value
  while IFS='=' read -r key value; do
    case "$key" in
      session) marker_session="$value" ;;
      token) marker_token="$value" ;;
    esac
  done < "$PLAYBACK_MARKER"
  [ "$marker_session" = "$SESSION_NAME" ] && [ -n "$marker_token" ] ||
    return 1
}

playback_session_owned() {
  playback_marker_valid || return 1
  tmux has-session -t "$SESSION_NAME" 2>/dev/null || return 1
  local marker_token="" key value tmux_token=""
  while IFS='=' read -r key value; do
    [ "$key" != "token" ] || marker_token="$value"
  done < "$PLAYBACK_MARKER"
  tmux_token="$(
    tmux show-options -v -t "$SESSION_NAME" "$PLAYBACK_TOKEN_OPTION" \
      2>/dev/null || true
  )"
  [ "$tmux_token" = "$marker_token" ]
}

stop_owned_master() {
  [ -f "$MASTER_MARKER" ] || return 0
  local marker_container="" marker_session="" token="" key value
  while IFS='=' read -r key value; do
    case "$key" in
      container) marker_container="$value" ;;
      session) marker_session="$value" ;;
      token) token="$value" ;;
    esac
  done < "$MASTER_MARKER"

  if [ "$marker_container" != "$CONTAINER_NAME" ] ||
    [ "$marker_session" != "$SESSION_NAME" ]; then
    echo "[WARN] Offline-master marker belongs to another container or playback session; leaving it untouched." >&2
    return 1
  fi
  if ! [[ "$token" =~ ^[A-Za-z0-9_.-]+$ ]]; then
    echo "[WARN] Offline-master ownership marker has an invalid token; leaving it untouched." >&2
    return 1
  fi

  if container_running; then
    docker exec -i -e "OWNED_MASTER_TOKEN=$token" "$CONTAINER_NAME" bash -lc '
      owned_master_pid() {
        local pid="$1" environ="/proc/$1/environ" cmdline="/proc/$1/cmdline"
        [ -r "$environ" ] && [ -r "$cmdline" ] || return 1
        tr "\0" "\n" < "$environ" 2>/dev/null |
          grep -Fxq "OFFLINE_BAG_MASTER_TOKEN=$OWNED_MASTER_TOKEN" || return 1
        tr "\0" "\n" < "$cmdline" 2>/dev/null |
          grep -Eq "(^|/)(roscore|rosmaster)$"
      }

      owned_pids=()
      for environ in /proc/[0-9]*/environ; do
        [ -r "$environ" ] || continue
        pid="$(basename "$(dirname "$environ")")"
        owned_master_pid "$pid" || continue
        owned_pids+=("$pid")
        kill -INT "$pid" 2>/dev/null || true
      done

      for _ in {1..20}; do
        remaining=false
        for pid in "${owned_pids[@]}"; do
          owned_master_pid "$pid" || continue
          remaining=true
        done
        [ "$remaining" = "true" ] || exit 0
        sleep 0.1
      done
      for pid in "${owned_pids[@]}"; do
        owned_master_pid "$pid" || continue
        kill -TERM "$pid" 2>/dev/null || true
      done
    ' || true
  fi
  rm -f -- "$MASTER_MARKER"
}

stop_owned_playback_nodes() {
  container_running || return 0
  if master_running; then
    docker_ros 'rosnode kill /offline_bag_player /offline_livox_converter /offline_body_to_livox_tf >/dev/null 2>&1 || true' || true
  fi
  docker exec -i "$CONTAINER_NAME" bash -lc '
    for cmdline in /proc/[0-9]*/cmdline; do
      [ -r "$cmdline" ] || continue
      argv=()
      mapfile -d "" -t argv < "$cmdline" 2>/dev/null || true
      owned=false
      for arg in "${argv[@]}"; do
        case "$arg" in
          __name:=offline_bag_player|__name:=offline_livox_converter|__name:=offline_body_to_livox_tf)
            owned=true
            ;;
        esac
      done
      if [ "$owned" = "true" ]; then
        pid="$(basename "$(dirname "$cmdline")")"
        kill -INT "$pid" 2>/dev/null || true
      fi
    done
  ' || true
}

stop_playback() {
  if ! tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
    if [ -f "$PLAYBACK_MARKER" ] && ! playback_marker_valid; then
      echo "[ERROR] Offline playback marker belongs to another session; leaving it untouched." >&2
      return 1
    fi
    if playback_marker_valid; then
      stop_owned_playback_nodes
      rm -f -- "$PLAYBACK_MARKER"
    fi
    stop_owned_master
    return 0
  fi
  if ! playback_session_owned; then
    echo "[ERROR] tmux session '$SESSION_NAME' is not owned by this playback launcher; leaving it untouched." >&2
    return 1
  fi
  stop_owned_playback_nodes
  tmux kill-session -t "$SESSION_NAME"
  stop_owned_master
  rm -f -- "$PLAYBACK_MARKER"
}

latest_bag() {
  [ -d "$RUNTIME_DIR/flight_bags" ] || return 1
  find "$RUNTIME_DIR/flight_bags" -maxdepth 1 -type f -name '*.bag' \
    -printf '%T@ %p\n' | sort -n | tail -n 1 | cut -d' ' -f2-
}

resolve_bag() {
  local requested="${1:-}" host_path bag_root
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
  [[ "$host_path" != *$'\n'* && "$host_path" != *$'\r'* ]] || {
    echo "[ERROR] Bag paths containing newlines are not supported." >&2
    return 1
  }
  host_path="$(realpath -e "$host_path")"
  bag_root="$(realpath -e "$RUNTIME_DIR/flight_bags")"
  case "$host_path" in
    "$bag_root"/*) printf '/root/flight_bags/%s\n' "${host_path#"$bag_root"/}" ;;
    *)
      echo "[ERROR] Bag must be inside $bag_root so the container can read it." >&2
      return 1
      ;;
  esac
}

docker_pane_command() {
  local inner_cmd="$1"
  shift
  local command arg_q
  printf -v command \
    'docker exec -e ROS_MASTER_URI=http://127.0.0.1:11312 -e ROS_IP=%q %q bash -lc %q bash' \
    "$ROS_IP" "$CONTAINER_NAME" "$inner_cmd"
  for arg in "$@"; do
    printf -v arg_q ' %q' "$arg"
    command+="$arg_q"
  done
  printf '%s\n' "$command"
}

play_bag() {
  local requested="${1:-}" bag_container master_owned=false master_token="" pane_cmd
  local playback_token
  playback_token="real_bag_playback_$$_$(date +%s%N)"
  need_cmd realpath
  need_cmd python3
  need_cmd ip
  if tmux has-session -t "$REAL_SESSION_NAME" 2>/dev/null; then
    echo "[ERROR] Real-flight stack '$REAL_SESSION_NAME' is running. Stop it before offline playback." >&2
    return 1
  fi
  container_running || {
    echo "[ERROR] Container '$CONTAINER_NAME' is not running." >&2
    return 1
  }
  [ -f "$CONVERTER_HOST" ] || {
    echo "[ERROR] Converter not found: $CONVERTER_HOST" >&2
    return 1
  }
  if ! [[ "$BAG_RATE" =~ ^([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]] ||
    ! python3 -c \
      'import math,sys; value=float(sys.argv[1]); raise SystemExit(0 if math.isfinite(value) and value > 0.0 else 1)' \
      "$BAG_RATE" 2>/dev/null; then
    echo "[ERROR] BAG_RATE must be a finite decimal number greater than zero." >&2
    return 1
  fi
  case "$BAG_LOOP" in
    true|false) ;;
    *)
      echo "[ERROR] BAG_LOOP must be true or false." >&2
      return 1
      ;;
  esac

  bag_container="$(resolve_bag "$requested")"
  [ -n "$ROS_IP" ] || ROS_IP="$(detect_ros_ip)"
  [ -n "$ROS_IP" ] || {
    echo "[ERROR] Unable to detect ROS_IP; set it explicitly." >&2
    return 1
  }

  stop_playback
  if flight_processes_running; then
    echo "[ERROR] Real-flight control, localization or recording processes are still running. Stop the real stack first." >&2
    return 1
  fi
  docker cp "$CONVERTER_HOST" "$CONTAINER_NAME:$CONVERTER_CONTAINER"
  docker exec "$CONTAINER_NAME" chmod +x "$CONVERTER_CONTAINER"

  if master_running; then
    if flight_nodes_running; then
      echo "[ERROR] A ROS master with real-flight nodes is already running. Stop the flight stack before playback." >&2
      return 1
    fi
    tmux new-session -d -s "$SESSION_NAME" -n external_master 'while sleep 3600; do :; done'
    echo "[INFO] Reusing the existing ROS master; it is not owned by this script."
  else
    master_token="real_bag_$$_$(date +%s%N)"
    write_master_marker "$master_token"
    printf -v pane_cmd \
      'docker exec -e ROS_MASTER_URI=http://127.0.0.1:11312 -e ROS_IP=%q -e OFFLINE_BAG_MASTER_TOKEN=%q %q bash -lc %q' \
      "$ROS_IP" "$master_token" "$CONTAINER_NAME" \
      'source /opt/ros/noetic/setup.bash; source /root/catkin_ws/devel/setup.bash; exec roscore -p 11312'
    tmux new-session -d -s "$SESSION_NAME" -n roscore "$pane_cmd"
    master_owned=true
  fi
  if ! claim_playback_session "$playback_token"; then
    echo "[ERROR] Failed to record ownership of offline playback session '$SESSION_NAME'." >&2
    tmux kill-session -t "$SESSION_NAME" >/dev/null 2>&1 || true
    stop_owned_master
    return 1
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
  if [ "$master_owned" != "true" ]; then
    rm -f -- "$MASTER_MARKER"
  fi
  docker_ros 'rosparam set /use_sim_time true'

  # Positional parameters are intentionally expanded by the container shell.
  # shellcheck disable=SC2016
  pane_cmd="$(docker_pane_command \
    'source /opt/ros/noetic/setup.bash; source /root/livox_ws/devel/setup.bash; source /root/catkin_ws/devel/setup.bash; exec python3 "$1" _input_topic:=/livox/lidar _output_topic:=/livox/lidar_points __name:=offline_livox_converter' \
    "$CONVERTER_CONTAINER")"
  tmux new-window -t "$SESSION_NAME" -n raw_converter "$pane_cmd"

  pane_cmd="$(docker_pane_command \
    'source /opt/ros/noetic/setup.bash; exec rosrun tf2_ros static_transform_publisher -0.011 -0.02329 0.04412 0 0 0 body livox_frame __name:=offline_body_to_livox_tf')"
  tmux new-window -t "$SESSION_NAME" -n lidar_tf "$pane_cmd"

  for _ in $(seq 1 20); do
    if docker_ros 'rosnode list | grep -qx /offline_livox_converter'; then
      break
    fi
    sleep 0.25
  done
  docker_ros 'rosnode list | grep -qx /offline_livox_converter' || {
    echo "[ERROR] Offline Livox converter did not start." >&2
    stop_playback
    return 1
  }

  # Positional parameters are intentionally expanded by the container shell.
  # shellcheck disable=SC2016
  pane_cmd="$(docker_pane_command \
    'source /opt/ros/noetic/setup.bash; source /root/livox_ws/devel/setup.bash; source /root/catkin_ws/devel/setup.bash; loop_args=(); [ "$3" != "true" ] || loop_args+=(--loop); exec rosbag play --clock --rate="$2" "${loop_args[@]}" "$1" __name:=offline_bag_player' \
    "$bag_container" "$BAG_RATE" "$BAG_LOOP")"
  tmux new-window -t "$SESSION_NAME" -n rosbag "$pane_cmd"

  echo "[INFO] Offline bag playback started."
  echo "[INFO] Bag: $bag_container"
  echo "[INFO] Rate: $BAG_RATE, loop: $BAG_LOOP"
  echo "[INFO] Raw-density display: /livox/lidar -> /livox/lidar_points"
  echo "[INFO] Session: $SESSION_NAME"
}

show_status() {
  if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
    playback_session_owned || {
      echo "[ERROR] tmux session '$SESSION_NAME' is not owned by this playback launcher." >&2
      return 1
    }
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
    play)
      [ "$#" -le 2 ] || {
        usage >&2
        exit 1
      }
      play_bag "${2:-}"
      ;;
    stop)
      [ "$#" -eq 1 ] || {
        usage >&2
        exit 1
      }
      stop_playback
      ;;
    status)
      [ "$#" -eq 1 ] || {
        usage >&2
        exit 1
      }
      show_status
      ;;
    attach)
      [ "$#" -eq 1 ] || {
        usage >&2
        exit 1
      }
      playback_session_owned || {
        echo "[ERROR] Offline playback is not running." >&2
        exit 1
      }
      exec tmux attach -t "$SESSION_NAME"
      ;;
    *)
      usage
      exit 1
      ;;
  esac
}

main "$@"
