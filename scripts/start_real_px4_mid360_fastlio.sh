#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME="${CONTAINER_NAME:-ros_noetic_realflight}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
PROJECT_NAME="$(basename "$PROJECT_ROOT")"
SESSION_NAME="${SESSION_NAME:-real_px4_stack}"
RUN_ID="${RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
LIVOX_LAUNCH="${LIVOX_LAUNCH:-msg_MID360s.launch}"
FASTLIO_RVIZ="${FASTLIO_RVIZ:-false}"
START_TIMEOUT="${START_TIMEOUT:-30}"
WAIT_INTERVAL="${WAIT_INTERVAL:-1}"
ODOM_RAW_TOPIC="${ODOM_RAW_TOPIC:-/Odometry}"
ODOM_BASE_TOPIC="${ODOM_BASE_TOPIC:-/Odometry_base}"
FCU_URL="${FCU_URL:-}"
GCS_URL="${GCS_URL:-}"
MOUNT_X="${MOUNT_X:-0.0}"
MOUNT_Y="${MOUNT_Y:-0.0}"
MOUNT_Z="${MOUNT_Z:-0.10}"
MOUNT_ROLL_DEG="${MOUNT_ROLL_DEG:-0.0}"
MOUNT_PITCH_DEG="${MOUNT_PITCH_DEG:-30.0}"
MOUNT_YAW_DEG="${MOUNT_YAW_DEG:-0.0}"
MAVROS_TGT_SYSTEM="${MAVROS_TGT_SYSTEM:-1}"
START_DIFF_PLANNER="${START_DIFF_PLANNER:-true}"
DIFF_PLANNER_LAUNCH="${DIFF_PLANNER_LAUNCH:-run_real_mid360_lio.launch}"
DIFF_PLANNER_INFLATION_SIZE="${DIFF_PLANNER_INFLATION_SIZE:-0.15}"
DIFF_PLANNER_MAX_VEL="${DIFF_PLANNER_MAX_VEL:-0.5}"
DIFF_PLANNER_MAX_ACC="${DIFF_PLANNER_MAX_ACC:-0.8}"
DIFF_PLANNER_MAX_JER="${DIFF_PLANNER_MAX_JER:-8.0}"
DIFF_PLANNER_YAW_DOT_MAX_DEG_S="${DIFF_PLANNER_YAW_DOT_MAX_DEG_S:-35.0}"
DIFF_PLANNER_YAW_DOT_DOT_MAX_DEG_S2="${DIFF_PLANNER_YAW_DOT_DOT_MAX_DEG_S2:-90.0}"
DIFF_PLANNER_POS_CMD_TOPIC="${DIFF_PLANNER_POS_CMD_TOPIC:-/drone_0_planning/pos_cmd}"
START_TRAJ_CONVERTER="${START_TRAJ_CONVERTER:-$START_DIFF_PLANNER}"
TRAJ_CONVERTER_INPUT_TOPIC="${TRAJ_CONVERTER_INPUT_TOPIC:-$DIFF_PLANNER_POS_CMD_TOPIC}"
TRAJ_CONVERTER_OUTPUT_TOPIC="${TRAJ_CONVERTER_OUTPUT_TOPIC:-/command/trajectory}"
START_SE3_CONTROLLER="${START_SE3_CONTROLLER:-$START_DIFF_PLANNER}"
SE3_ENABLE_SIM="${SE3_ENABLE_SIM:-false}"
SE3_AUTO_REQUEST_OFFBOARD="${SE3_AUTO_REQUEST_OFFBOARD:-false}"
SE3_AUTO_REQUEST_ARM="${SE3_AUTO_REQUEST_ARM:-false}"
SE3_AUTO_LAND_ON_GEOFENCE="${SE3_AUTO_LAND_ON_GEOFENCE:-false}"
SE3_TAKEOFF_HEIGHT="${SE3_TAKEOFF_HEIGHT:-0.3}"
SE3_ENABLE_THRUST_ESTIMATION="${SE3_ENABLE_THRUST_ESTIMATION:-false}"
SE3_USE_ACCELERATION_FEEDFORWARD="${SE3_USE_ACCELERATION_FEEDFORWARD:-true}"
SE3_USE_YAW_RATE_FEEDFORWARD="${SE3_USE_YAW_RATE_FEEDFORWARD:-true}"
SE3_MAX_FEEDFORWARD_ACC="${SE3_MAX_FEEDFORWARD_ACC:-2.0}"
SE3_HOVER_PERCENT="${SE3_HOVER_PERCENT:-0.45}"
SE3_MAX_HOVER_PERCENT="${SE3_MAX_HOVER_PERCENT:-0.75}"
SE3_MIN_OUTPUT_THRUST="${SE3_MIN_OUTPUT_THRUST:-0.20}"
SE3_MAX_OUTPUT_THRUST="${SE3_MAX_OUTPUT_THRUST:-0.85}"
SE3_GEOFENCE_X="${SE3_GEOFENCE_X:-2.0}"
SE3_GEOFENCE_Y="${SE3_GEOFENCE_Y:-2.0}"
SE3_GEOFENCE_Z="${SE3_GEOFENCE_Z:-1.5}"
SE3_NODE_NAME="${SE3_NODE_NAME:-/se3_controller_node}"
MAVROS_ATTITUDE_TOPIC="${MAVROS_ATTITUDE_TOPIC:-/mavros/setpoint_raw/attitude}"
ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
ROS_IP="${ROS_IP:-}"
HOST_LOG_DIR="${HOST_LOG_DIR:-$HOME/${PROJECT_NAME}_logs/$RUN_ID}"
CONTAINER_ROS_LOG_DIR="${CONTAINER_ROS_LOG_DIR:-/root/flight_bags/ros_logs/$RUN_ID}"
PROCESS_GREP_PATTERN="roscore|rosmaster|roslaunch livox_ros_driver2|livox_ros_driver2_node|roslaunch fast_lio mapping_mid360.launch|fastlio_mapping|roslaunch mavros px4.launch|mavros_node|roslaunch diff_planner|traj_server|ego_planner|plan_manage|trajectory_msg_converter.py|se3_controller_node|odom_to_base.py|odom_to_pose.py"

usage() {
  cat <<'EOF'
Usage: start_real_px4_mid360_fastlio.sh {start|stop|restart|status|attach}

Default assumptions for this staged copy:
  - container name: ros_noetic_realflight
  - container was created by docker/docker_run_real.sh
EOF
}

need_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "[ERROR] Missing command: $1"
    exit 1
  fi
}

ensure_prereqs() {
  need_cmd tmux
  need_cmd docker
}

detect_host_ip() {
  local target="${ROS_IP_TARGET:-10.0.30.196}"
  local detected

  detected="$(ip route get "$target" 2>/dev/null | awk '{for (i=1; i<=NF; i++) if ($i == "src") {print $(i+1); exit}}')"
  if [ -z "$detected" ]; then
    detected="$(hostname -I 2>/dev/null | awk '{print $1}')"
  fi

  echo "$detected"
}

resolve_ros_ip() {
  if [ -z "$ROS_IP" ]; then
    ROS_IP="$(detect_host_ip)"
  fi

  if [ -z "$ROS_IP" ]; then
    echo "[ERROR] Failed to detect ROS_IP. Set ROS_IP manually and retry."
    exit 1
  fi
}

tmux_has_session() {
  tmux has-session -t "$SESSION_NAME" >/dev/null 2>&1
}

tmux_has_window() {
  local window_name="$1"
  tmux list-windows -t "$SESSION_NAME" -F '#W' 2>/dev/null | grep -qx "$window_name"
}

docker_container_exists() {
  docker inspect "$CONTAINER_NAME" >/dev/null 2>&1
}

docker_container_running() {
  [ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER_NAME" 2>/dev/null || echo false)" = "true" ]
}

ensure_container_exists() {
  if ! docker_container_exists; then
    echo "[ERROR] Docker container '$CONTAINER_NAME' does not exist."
    echo "Please create it first with docker/docker_run_real.sh run."
    exit 1
  fi
}

ensure_container_running() {
  ensure_container_exists
  if ! docker_container_running; then
    echo "[INFO] Starting container: $CONTAINER_NAME"
    docker start "$CONTAINER_NAME" >/dev/null
  fi
}

docker_exec_shell() {
  local inner_cmd="$1"
  docker exec -i \
    -e "ROS_MASTER_URI=$ROS_MASTER_URI" \
    -e "ROS_IP=$ROS_IP" \
    -e "ROS_LOG_DIR=$CONTAINER_ROS_LOG_DIR" \
    "$CONTAINER_NAME" bash -lc "unset ROS_HOSTNAME && source ~/.bashrc && mkdir -p \"\$ROS_LOG_DIR\" && $inner_cmd"
}

docker_tmux_cmd() {
  local inner_cmd="$1"
  printf "docker exec -it -e ROS_MASTER_URI=%q -e ROS_IP=%q -e ROS_LOG_DIR=%q %q bash -lc %q" \
    "$ROS_MASTER_URI" \
    "$ROS_IP" \
    "$CONTAINER_ROS_LOG_DIR" \
    "$CONTAINER_NAME" \
    "unset ROS_HOSTNAME && source ~/.bashrc && mkdir -p \"\$ROS_LOG_DIR\" && $inner_cmd"
}

enable_window_logging() {
  local window_name="$1"

  mkdir -p "$HOST_LOG_DIR"
  tmux pipe-pane -t "$SESSION_NAME:$window_name" -o "cat >> '$HOST_LOG_DIR/$window_name.tmux.log'"
}

container_processes_running() {
  if ! docker_container_exists || ! docker_container_running; then
    return 1
  fi

  docker exec -i "$CONTAINER_NAME" bash -lc "pgrep -af \"$PROCESS_GREP_PATTERN\" | grep -v 'pgrep -af' >/dev/null"
}

cleanup_container_processes() {
  if ! docker_container_exists || ! docker_container_running; then
    return 0
  fi

  docker exec -i "$CONTAINER_NAME" bash -lc '
    kill_matching() {
      local signal="$1"
      local pattern="$2"
      while read -r pid; do
        [ -n "$pid" ] || continue
        [ "$pid" = "$$" ] && continue
        [ "$pid" = "$PPID" ] && continue
        kill "-$signal" "$pid" 2>/dev/null || true
      done < <(pgrep -f "$pattern" || true)
    }

    for pattern in \
      "roscore" \
      "rosmaster" \
      "roslaunch livox_ros_driver2" \
      "roslaunch fast_lio mapping_mid360.launch" \
      "roslaunch mavros px4.launch" \
      "roslaunch diff_planner" \
      "trajectory_msg_converter.py" \
      "se3_controller_node" \
      "odom_to_base.py" \
      "odom_to_pose.py"; do
      kill_matching INT "$pattern"
    done

    sleep 2

    for pattern in \
      "roscore" \
      "rosmaster" \
      "livox_ros_driver2_node" \
      "fastlio_mapping" \
      "mavros_node" \
      "traj_server" \
      "ego_planner" \
      "plan_manage" \
      "trajectory_msg_converter.py" \
      "se3_controller_node" \
      "odom_to_base.py" \
      "odom_to_pose.py" \
      "roslaunch livox_ros_driver2" \
      "roslaunch fast_lio mapping_mid360.launch" \
      "roslaunch mavros px4.launch" \
      "roslaunch diff_planner"; do
      kill_matching TERM "$pattern"
    done

    sleep 2

    for pattern in \
      "roscore" \
      "rosmaster" \
      "livox_ros_driver2_node" \
      "fastlio_mapping" \
      "mavros_node" \
      "traj_server" \
      "ego_planner" \
      "plan_manage" \
      "trajectory_msg_converter.py" \
      "se3_controller_node" \
      "odom_to_base.py" \
      "odom_to_pose.py" \
      "roslaunch livox_ros_driver2" \
      "roslaunch fast_lio mapping_mid360.launch" \
      "roslaunch mavros px4.launch" \
      "roslaunch diff_planner"; do
      kill_matching KILL "$pattern"
    done
  '

  local waited=0
  while container_processes_running; do
    if [ "$waited" -ge 15 ]; then
      echo "[WARN] Some container processes are still running after cleanup:"
      docker exec -i "$CONTAINER_NAME" bash -lc "pgrep -af \"$PROCESS_GREP_PATTERN\" | grep -v 'pgrep -af' || true"
      break
    fi
    sleep 1
    waited=$((waited + 1))
  done
}

wait_for_condition() {
  local stage="$1"
  local check_cmd="$2"
  local waited=0

  echo "[INFO] Waiting for $stage ..."
  while true; do
    if docker_exec_shell "$check_cmd" >/dev/null 2>&1; then
      echo "[INFO] $stage is ready."
      return 0
    fi

    if [ "$waited" -ge "$START_TIMEOUT" ]; then
      echo "[ERROR] Timed out while waiting for $stage."
      echo "Run '$0 status' or '$0 attach' to inspect current logs."
      echo "Host tmux logs: $HOST_LOG_DIR"
      echo "Container ROS logs: $CONTAINER_ROS_LOG_DIR"
      return 1
    fi

    sleep "$WAIT_INTERVAL"
    waited=$((waited + WAIT_INTERVAL))
    echo "[INFO] Still waiting for $stage ... ${waited}/${START_TIMEOUT}s"
  done
}

create_window() {
  local window_name="$1"
  local inner_cmd="$2"

  tmux new-window -t "$SESSION_NAME" -n "$window_name" "$(docker_tmux_cmd "$inner_cmd")"
  enable_window_logging "$window_name"
}

start_stack() {
  ensure_prereqs
  resolve_ros_ip
  ensure_container_running

  if tmux_has_session; then
    echo "[INFO] tmux session '$SESSION_NAME' already exists."
    echo "Use '$0 attach' to view it or '$0 restart' to recreate it."
    exit 0
  fi

  echo "[INFO] Creating tmux session: $SESSION_NAME"
  echo "[INFO] ROS_MASTER_URI=$ROS_MASTER_URI"
  echo "[INFO] ROS_IP=$ROS_IP"
  echo "[INFO] Host tmux logs: $HOST_LOG_DIR"
  echo "[INFO] Container ROS logs: $CONTAINER_ROS_LOG_DIR"

  tmux new-session -d -s "$SESSION_NAME" -n roscore "$(docker_tmux_cmd "roscore")"
  tmux set-option -t "$SESSION_NAME" remain-on-exit on
  tmux set-option -t "$SESSION_NAME" history-limit 20000
  enable_window_logging "roscore"

  wait_for_condition "ROS master" "rosparam get /run_id >/dev/null"

  create_window "mid360" \
    "source /opt/ros/noetic/setup.bash && source ~/livox_ws/devel/setup.bash && roslaunch livox_ros_driver2 $LIVOX_LAUNCH"

  wait_for_condition "Mid-360S topics" "rostopic list | grep -q '^/livox/lidar$' && rostopic list | grep -q '^/livox/imu$' && [ \"\$(rostopic type /livox/lidar)\" = 'livox_ros_driver2/CustomMsg' ]"

  create_window "fast_lio" \
    "source ~/.bashrc && roslaunch fast_lio mapping_mid360.launch rviz:=$FASTLIO_RVIZ"

  wait_for_condition "FAST-LIO odometry" "rostopic list | grep -qx '$ODOM_RAW_TOPIC'"

  create_window "odom_to_base" \
    "source ~/.bashrc && rosrun px4_realflight_tools odom_to_base.py _input_topic:=$ODOM_RAW_TOPIC _output_topic:=$ODOM_BASE_TOPIC _mount_x:=$MOUNT_X _mount_y:=$MOUNT_Y _mount_z:=$MOUNT_Z _mount_roll_deg:=$MOUNT_ROLL_DEG _mount_pitch_deg:=$MOUNT_PITCH_DEG _mount_yaw_deg:=$MOUNT_YAW_DEG"

  wait_for_condition "base_link odometry" "rostopic list | grep -qx '$ODOM_BASE_TOPIC'"

  tmux new-window -t "$SESSION_NAME" -n mavros \
    "docker exec -it -e ROS_MASTER_URI='$ROS_MASTER_URI' -e ROS_IP='$ROS_IP' -e ROS_LOG_DIR='$CONTAINER_ROS_LOG_DIR' -e HOST_FCU_URL='$FCU_URL' -e HOST_GCS_URL='$GCS_URL' '$CONTAINER_NAME' bash -lc 'unset ROS_HOSTNAME && source ~/.bashrc && mkdir -p \"\$ROS_LOG_DIR\" && if [ -n \"\$HOST_FCU_URL\" ]; then export FCU_URL=\"\$HOST_FCU_URL\"; fi && if [ -n \"\$HOST_GCS_URL\" ]; then export GCS_URL=\"\$HOST_GCS_URL\"; fi && roslaunch mavros px4.launch fcu_url:=\"\$FCU_URL\" gcs_url:=\"\$GCS_URL\" tgt_system:=$MAVROS_TGT_SYSTEM'"
  enable_window_logging "mavros"

  wait_for_condition "MAVROS connection" "rostopic list | grep -q '^/mavros/state$' && timeout 3s rostopic echo -n 1 /mavros/state | grep -q 'connected: True'"

  create_window "odom_to_pose" \
    "source ~/.bashrc && rosrun px4_realflight_tools odom_to_pose.py _odom_topic:=$ODOM_BASE_TOPIC _pose_topic:=/mavros/vision_pose/pose _frame_id:=map _publish_rate:=30.0 _max_input_age:=0.2 _use_input_stamp:=false"

  wait_for_condition "vision pose bridge" "rostopic list | grep -q '^/mavros/vision_pose/pose$'"

  if [ "$START_DIFF_PLANNER" = "true" ]; then
    create_window "diff_planner" \
      "source ~/.bashrc && roslaunch diff_planner $DIFF_PLANNER_LAUNCH inflation_size:=$DIFF_PLANNER_INFLATION_SIZE max_vel:=$DIFF_PLANNER_MAX_VEL max_acc:=$DIFF_PLANNER_MAX_ACC max_jer:=$DIFF_PLANNER_MAX_JER yaw_dot_max_deg_s:=$DIFF_PLANNER_YAW_DOT_MAX_DEG_S yaw_dot_dot_max_deg_s2:=$DIFF_PLANNER_YAW_DOT_DOT_MAX_DEG_S2"

    wait_for_condition "Diff-Planner" "rostopic list | grep -qx '$DIFF_PLANNER_POS_CMD_TOPIC' || rosnode list | grep -q 'traj_server'"
  else
    echo "[INFO] Diff-Planner startup skipped because START_DIFF_PLANNER=false."
  fi

  if [ "$START_TRAJ_CONVERTER" = "true" ]; then
    create_window "traj_converter" \
      "source ~/.bashrc && rosrun diff_planner trajectory_msg_converter.py _traj_topic:=$TRAJ_CONVERTER_INPUT_TOPIC _traj_pub_topic:=$TRAJ_CONVERTER_OUTPUT_TOPIC"

    wait_for_condition "trajectory converter" "rosnode list | grep -qx '/trajectory_msg_converter' && rostopic list | grep -qx '$TRAJ_CONVERTER_OUTPUT_TOPIC'"
  else
    echo "[INFO] trajectory converter startup skipped because START_TRAJ_CONVERTER=false."
  fi

  if [ "$START_SE3_CONTROLLER" = "true" ]; then
    create_window "se3_controller" \
      "source ~/.bashrc && rosparam set /se3_controller_node/enable_sim $SE3_ENABLE_SIM && rosparam set /se3_controller_node/auto_request_offboard $SE3_AUTO_REQUEST_OFFBOARD && rosparam set /se3_controller_node/auto_request_arm $SE3_AUTO_REQUEST_ARM && rosparam set /se3_controller_node/auto_land_on_geofence $SE3_AUTO_LAND_ON_GEOFENCE && rosparam set /se3_controller_node/takeoff_height $SE3_TAKEOFF_HEIGHT && rosparam set /se3_controller_node/enable_thrust_estimation $SE3_ENABLE_THRUST_ESTIMATION && rosparam set /se3_controller_node/use_acceleration_feedforward $SE3_USE_ACCELERATION_FEEDFORWARD && rosparam set /se3_controller_node/use_yaw_rate_feedforward $SE3_USE_YAW_RATE_FEEDFORWARD && rosparam set /se3_controller_node/max_feedforward_acc $SE3_MAX_FEEDFORWARD_ACC && rosparam set /se3_controller_node/hover_percent $SE3_HOVER_PERCENT && rosparam set /se3_controller_node/max_hover_percent $SE3_MAX_HOVER_PERCENT && rosparam set /se3_controller_node/min_output_thrust $SE3_MIN_OUTPUT_THRUST && rosparam set /se3_controller_node/max_output_thrust $SE3_MAX_OUTPUT_THRUST && rosparam set /se3_controller_node/geo_fence/x $SE3_GEOFENCE_X && rosparam set /se3_controller_node/geo_fence/y $SE3_GEOFENCE_Y && rosparam set /se3_controller_node/geo_fence/z $SE3_GEOFENCE_Z && rosrun se3_controller se3_controller_node"

    wait_for_condition "SE3 controller" "rosnode list | grep -qx '$SE3_NODE_NAME' && rostopic list | grep -qx '$MAVROS_ATTITUDE_TOPIC'"
  else
    echo "[INFO] SE3 controller startup skipped because START_SE3_CONTROLLER=false."
  fi

  echo "[INFO] Real-flight stack started successfully."
  echo "[INFO] Use '$0 attach' to inspect tmux windows."
  echo "[INFO] Host tmux logs: $HOST_LOG_DIR"
  echo "[INFO] Container ROS logs: $CONTAINER_ROS_LOG_DIR"
}

stop_stack() {
  ensure_prereqs

  echo "[INFO] Stopping real-flight stack..."

  if tmux_has_session; then
    for window_name in se3_controller traj_converter diff_planner odom_to_pose mavros odom_to_base fast_lio mid360 roscore; do
      if tmux_has_window "$window_name"; then
        tmux send-keys -t "$SESSION_NAME:$window_name" C-c
      fi
    done

    sleep 3
    tmux kill-session -t "$SESSION_NAME" || true
  else
    echo "[INFO] tmux session '$SESSION_NAME' is not running."
  fi

  cleanup_container_processes

  if tmux_has_session; then
    echo "[WARN] tmux session '$SESSION_NAME' still exists after stop attempt."
  else
    echo "[INFO] tmux session stopped: $SESSION_NAME"
  fi

  if container_processes_running; then
    echo "[WARN] Residual container processes are still running."
  else
    echo "[INFO] Container processes stopped cleanly."
  fi
}

status_stack() {
  ensure_prereqs

  echo "[INFO] Container status:"
  if docker_container_exists; then
    docker ps -a --filter "name=^/${CONTAINER_NAME}$" --format 'table {{.Names}}\t{{.Status}}'
  else
    echo "[ERROR] Docker container '$CONTAINER_NAME' does not exist."
  fi

  if tmux_has_session; then
    echo
    echo "[INFO] tmux windows:"
    tmux list-windows -t "$SESSION_NAME"
    echo
    echo "[INFO] tmux pane states:"
    tmux list-panes -a -F '#{window_name}: #{pane_current_command}'
  else
    echo
    echo "[INFO] tmux session '$SESSION_NAME' is not running."
  fi

  echo
  echo "[INFO] Host tmux logs root: ${HOST_LOG_DIR%/*}"
}

attach_stack() {
  ensure_prereqs

  if ! tmux_has_session; then
    echo "[ERROR] tmux session '$SESSION_NAME' is not running."
    exit 1
  fi

  exec tmux attach -t "$SESSION_NAME"
}

main() {
  local action="${1:-}"

  case "$action" in
    start)
      start_stack
      ;;
    stop)
      stop_stack
      ;;
    restart)
      stop_stack
      start_stack
      ;;
    status)
      status_stack
      ;;
    attach)
      attach_stack
      ;;
    *)
      usage
      exit 1
      ;;
  esac
}

main "$@"
