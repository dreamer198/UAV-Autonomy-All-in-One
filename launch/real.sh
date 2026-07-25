#!/usr/bin/env bash
set -Eeuo pipefail

CONTAINER_NAME="${CONTAINER_NAME:-diff_planner_px4_real}"
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
LOCALIZATION_ODOM_TOPIC="${LOCALIZATION_ODOM_TOPIC:-/localization/odom}"
RAW_REGISTERED_CLOUD_TOPIC="${RAW_REGISTERED_CLOUD_TOPIC:-/cloud_registered}"
LOCALIZATION_CLOUD_TOPIC="${LOCALIZATION_CLOUD_TOPIC:-/localization/cloud_registered}"
FCU_URL="${FCU_URL:-}"
GCS_URL="${GCS_URL:-}"
# T_base_fastlio_body. FAST-LIO's body is the MID-360 internal IMU origin;
# these are the calibrated defaults for the current airframe.
MOUNT_X="${MOUNT_X:-0.109}"
MOUNT_Y="${MOUNT_Y:-0.024}"
MOUNT_Z="${MOUNT_Z:-0.006}"
MOUNT_ROLL_DEG="${MOUNT_ROLL_DEG:-0.7}"
MOUNT_PITCH_DEG="${MOUNT_PITCH_DEG:-28.1}"
MOUNT_YAW_DEG="${MOUNT_YAW_DEG:-0.5}"
MAVROS_TGT_SYSTEM="${MAVROS_TGT_SYSTEM:-5}"
DRONE_ID="${DRONE_ID:-0}"
START_DIFF_PLANNER="${START_DIFF_PLANNER:-true}"
DIFF_PLANNER_POS_CMD_TOPIC="${DIFF_PLANNER_POS_CMD_TOPIC:-/drone_${DRONE_ID}_planning/pos_cmd}"
DIFF_PLANNER_TRAJECTORY_TOPIC="${DIFF_PLANNER_TRAJECTORY_TOPIC:-/drone_${DRONE_ID}_planning/trajectory}"
DIFF_PLANNER_DATA_DISPLAY_TOPIC="${DIFF_PLANNER_DATA_DISPLAY_TOPIC:-/drone_${DRONE_ID}_planning/data_display}"
DIFF_PLANNER_INFLATED_MAP_TOPIC="${DIFF_PLANNER_INFLATED_MAP_TOPIC:-/drone_${DRONE_ID}_diff_planner_node/grid_map/occupancy_inflate}"
START_TRAJ_CONVERTER="${START_TRAJ_CONVERTER:-$START_DIFF_PLANNER}"
TRAJ_CONVERTER_OUTPUT_TOPIC="${TRAJ_CONVERTER_OUTPUT_TOPIC:-/command/trajectory}"
START_SE3_CONTROLLER="${START_SE3_CONTROLLER:-$START_DIFF_PLANNER}"
PLANNER_CONFIG="${PLANNER_CONFIG:-}"
PLANNER_RESOLUTION="${PLANNER_RESOLUTION:-}"
PLANNER_OBSTACLES_INFLATION="${PLANNER_OBSTACLES_INFLATION:-}"
CONTROLLER_CONFIG="${CONTROLLER_CONFIG:-/root/deployment/controller.yaml}"
SE3_NODE_NAME="${SE3_NODE_NAME:-/se3_controller_node}"
MAVROS_ATTITUDE_TOPIC="${MAVROS_ATTITUDE_TOPIC:-/mavros/setpoint_raw/attitude}"
# 自动录包：默认记录控制、定位、规划输入输出、原始 /livox/lidar、
# 较高密度的去畸变 /cloud_registered_body 和 2 Hz 膨胀地图。
# bag 落在容器内 $ROSBAG_DIR，经 real_container.sh 的 bind-mount 持久化到宿主
# ~/<project>/runtime/flight_bags/。关闭录制：START_ROSBAG=false。
START_ROSBAG="${START_ROSBAG:-true}"
ROSBAG_DIR="${ROSBAG_DIR:-/root/flight_bags}"
ROSBAG_PREFIX="${ROSBAG_PREFIX:-se3_test}"
ROSBAG_NODE_NAME="${ROSBAG_NODE_NAME:-/flight_recorder}"
ROSBAG_TOPICS="${ROSBAG_TOPICS:-/tf /tf_static $ODOM_RAW_TOPIC $LOCALIZATION_ODOM_TOPIC $LOCALIZATION_CLOUD_TOPIC /cloud_registered_body /mavros/vision_pose/pose /livox/imu /mavros/local_position/odom /mavros/local_position/pose /mavros/imu/data /mavros/state /mavros/battery /mavros/altitude /mavros/rc/in $MAVROS_ATTITUDE_TOPIC /mavros/setpoint_raw/target_attitude /mavros/setpoint_position/local $TRAJ_CONVERTER_OUTPUT_TOPIC /desire_odom_pub $DIFF_PLANNER_POS_CMD_TOPIC $DIFF_PLANNER_TRAJECTORY_TOPIC $DIFF_PLANNER_DATA_DISPLAY_TOPIC $DIFF_PLANNER_INFLATED_MAP_TOPIC /goal}"
ROSBAG_EXTRA_ARGS="${ROSBAG_EXTRA_ARGS:-}"
ROSBAG_RECORD_RAW_LIDAR="${ROSBAG_RECORD_RAW_LIDAR:-true}"
ROSBAG_SPLIT_SIZE_MB="${ROSBAG_SPLIT_SIZE_MB:-5120}"
ROSBAG_NICE_LEVEL="${ROSBAG_NICE_LEVEL:-10}"
ROSBAG_MIN_FREE_GB="${ROSBAG_MIN_FREE_GB:-5}"
ROSBAG_STOP_TIMEOUT="${ROSBAG_STOP_TIMEOUT:-60}"
ROSBAG_STATE_FILE="${ROSBAG_STATE_FILE:-$ROSBAG_DIR/.active_recording_prefix}"
REAL_COMMAND_TIMEOUT="${REAL_COMMAND_TIMEOUT:-15}"
REAL_TAKEOFF_HEIGHT="${REAL_TAKEOFF_HEIGHT:-1.0}"
REAL_TAKEOFF_TIMEOUT="${REAL_TAKEOFF_TIMEOUT:-30}"
REAL_TAKEOFF_TOLERANCE="${REAL_TAKEOFF_TOLERANCE:-0.1}"
REAL_TAKEOFF_STABLE_TIME="${REAL_TAKEOFF_STABLE_TIME:-0.5}"
REAL_TAKEOFF_MAX_VERTICAL_SPEED="${REAL_TAKEOFF_MAX_VERTICAL_SPEED:-0.2}"
MISSION_RUNNER_HOST="$PROJECT_ROOT/common/scripts/waypoint_mission.py"
MISSION_EXECUTOR_HOST="$PROJECT_ROOT/common/scripts/mission_executor.py"
ARM_EXECUTOR_HOST="$PROJECT_ROOT/common/scripts/arm_executor.py"
GOAL_EXECUTOR_HOST="$PROJECT_ROOT/common/scripts/goal_executor.py"
REAL_PREFLIGHT_TIMEOUT="${REAL_PREFLIGHT_TIMEOUT:-5.0}"
if [ "$ROSBAG_RECORD_RAW_LIDAR" = "true" ] && [[ " $ROSBAG_TOPICS " != *" /livox/lidar "* ]]; then
  ROSBAG_TOPICS+=" /livox/lidar"
fi
ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
ROS_IP="${ROS_IP:-}"
HOST_LOG_DIR="${HOST_LOG_DIR:-$HOME/${PROJECT_NAME}_logs/$RUN_ID}"
CONTAINER_ROS_LOG_DIR="${CONTAINER_ROS_LOG_DIR:-/root/flight_bags/ros_logs/$RUN_ID}"
PROCESS_GREP_PATTERN="roscore|rosmaster|roslaunch livox_ros_driver2|livox_ros_driver2_node|roslaunch fast_lio mapping_mid360.launch|fastlio_mapping|roslaunch mavros px4.launch|mavros_node|roslaunch sim2real_common|roslaunch sim2real_deployment frame_aliases.launch|static_transform_publisher.*real_world_|traj_server|diff_planner_node|trajectory_msg_converter.py|se3_controller_node|localization_guard.py|odom_to_base.py|odom_to_pose.py|cloud_relay.py|__name:=flight_recorder"

usage() {
  cat <<'EOF'
Usage: real.sh {start|stop|restart|status|attach|arm|land|goal X Y Z [YAW_DEG]|mission FILE}

Default assumptions for this staged copy:
  - container name: diff_planner_px4_real
  - container was created by launch/real_container.sh

Flight commands:
  arm                   Set PX4's takeoff height, request arming, and enter
                        AUTO.TAKEOFF, then verified OFFBOARD hold.
  goal X Y Z [YAW_DEG]  Publish an armed/OFFBOARD planning goal in world.
                        Omit YAW_DEG to leave the final yaw unconstrained.
  mission FILE          Execute ordered JSON waypoints. If disarmed, perform
                        native takeoff and automatic OFFBOARD first; land only
                        after every waypoint succeeds. RC mode change aborts.
  land                  Request PX4 AUTO.LAND; never force-disarms.
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

validate_rosbag_settings() {
  if [ "$START_ROSBAG" != "true" ]; then
    return
  fi

  if [ "$ROSBAG_RECORD_RAW_LIDAR" != "true" ] && [ "$ROSBAG_RECORD_RAW_LIDAR" != "false" ]; then
    echo "[ERROR] ROSBAG_RECORD_RAW_LIDAR must be true or false." >&2
    return 1
  fi
  if ! [[ "$ROSBAG_SPLIT_SIZE_MB" =~ ^[1-9][0-9]*$ ]]; then
    echo "[ERROR] ROSBAG_SPLIT_SIZE_MB must be a positive integer." >&2
    return 1
  fi
  if ! [[ "$ROSBAG_NICE_LEVEL" =~ ^([0-9]|1[0-9])$ ]]; then
    echo "[ERROR] ROSBAG_NICE_LEVEL must be an integer from 0 to 19." >&2
    return 1
  fi
  if ! [[ "$ROSBAG_MIN_FREE_GB" =~ ^[0-9]+$ ]]; then
    echo "[ERROR] ROSBAG_MIN_FREE_GB must be a non-negative integer." >&2
    return 1
  fi
  if ! [[ "$ROSBAG_STOP_TIMEOUT" =~ ^[1-9][0-9]*$ ]]; then
    echo "[ERROR] ROSBAG_STOP_TIMEOUT must be a positive integer." >&2
    return 1
  fi
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
    echo "Please create it first with launch/real_container.sh run."
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

rosbag_record_process_running() {
  docker_container_exists && docker_container_running && \
    docker exec -i "$CONTAINER_NAME" bash -lc \
      "pgrep -f '^/opt/ros/noetic/lib/rosbag/record ' >/dev/null"
}

current_rosbag_output_prefix() {
  if ! docker_container_exists || ! docker_container_running; then
    return 1
  fi

  docker exec -i \
    -e "ROSBAG_STATE_FILE=$ROSBAG_STATE_FILE" \
    "$CONTAINER_NAME" bash -lc '
      pid="$(pgrep -f "^/opt/ros/noetic/lib/rosbag/record " | head -n 1)"
      if [ -n "$pid" ]; then
        expect_prefix=false
        while IFS= read -r -d "" arg; do
          if [ "$expect_prefix" = "true" ]; then
            printf "%s\n" "$arg"
            exit 0
          fi
          if [ "$arg" = "-O" ] || [ "$arg" = "--output-prefix" ]; then
            expect_prefix=true
          fi
        done < "/proc/$pid/cmdline"
      fi
      if [ -s "$ROSBAG_STATE_FILE" ]; then
        head -n 1 "$ROSBAG_STATE_FILE"
        exit 0
      fi
      exit 1
    '
}

write_rosbag_state() {
  local output_prefix="$1"
  docker exec -i \
    -e "RECORDING_PREFIX=$output_prefix" \
    -e "ROSBAG_STATE_FILE=$ROSBAG_STATE_FILE" \
    "$CONTAINER_NAME" bash -lc '
      mkdir -p "$(dirname "$ROSBAG_STATE_FILE")"
      printf "%s\n" "$RECORDING_PREFIX" > "$ROSBAG_STATE_FILE"
    '
}

finalize_indexed_active_bags() {
  local output_prefix="$1"
  [ -n "$output_prefix" ] || return 0

  docker exec -i \
    -e "BAG_PREFIX=$output_prefix" \
    -e "ROSBAG_STATE_FILE=$ROSBAG_STATE_FILE" \
    "$CONTAINER_NAME" bash -lc '
      source /opt/ros/noetic/setup.bash
      failed=false
      for active in "${BAG_PREFIX}"_*.bag.active; do
        [ -e "$active" ] || continue
        final="${active%.active}"
        if [ -e "$final" ]; then
          echo "[WARN] Cannot finalize $active because $final already exists." >&2
          failed=true
        elif rosbag info "$active" >/dev/null 2>&1; then
          mv -- "$active" "$final"
          echo "[INFO] Finalized indexed bag: $final"
        else
          echo "[WARN] Bag is not fully indexed; leaving it for rosbag reindex: $active" >&2
          failed=true
        fi
      done
      if [ "$failed" = "false" ]; then
        rm -f "$ROSBAG_STATE_FILE"
      fi
      [ "$failed" = "false" ]
    '
}

stop_rosbag_gracefully() {
  local waited=0 timeout="$ROSBAG_STOP_TIMEOUT"
  local output_prefix=""

  if ! [[ "$timeout" =~ ^[1-9][0-9]*$ ]]; then
    echo "[WARN] Invalid ROSBAG_STOP_TIMEOUT=$timeout; using 60 seconds."
    timeout=60
  fi
  if ! docker_container_exists || ! docker_container_running; then
    return 0
  fi

  output_prefix="$(current_rosbag_output_prefix 2>/dev/null || true)"
  if ! rosbag_record_process_running; then
    finalize_indexed_active_bags "$output_prefix" || true
    return 0
  fi

  echo "[INFO] Requesting rosbag recorder shutdown; timeout ${timeout}s ..."
  # Sending C-c to the tmux pane can stop only the host-side `docker exec`
  # client while leaving rosbag alive in the container. Signal rosbag itself.
  docker exec -i "$CONTAINER_NAME" bash -lc '
    while read -r pid; do
      [ -n "$pid" ] && kill -INT "$pid" 2>/dev/null || true
    done < <(pgrep -f "^/opt/ros/noetic/lib/rosbag/record " || true)
  '

  while rosbag_record_process_running; do
    if [ "$waited" -ge "$timeout" ]; then
      echo "[WARN] rosbag did not exit within ${timeout}s; continuing forced cleanup." >&2
      return 1
    fi
    sleep 1
    waited=$((waited + 1))
  done

  echo "[INFO] rosbag recorder exited cleanly after ${waited}s."
  finalize_indexed_active_bags "$output_prefix" || true
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
      "__name:=flight_recorder" \
      "roscore" \
      "rosmaster" \
      "roslaunch livox_ros_driver2" \
      "roslaunch fast_lio mapping_mid360.launch" \
      "roslaunch mavros px4.launch" \
      "roslaunch sim2real_common" \
      "roslaunch sim2real_deployment frame_aliases.launch" \
      "static_transform_publisher.*real_world_" \
      "trajectory_msg_converter.py" \
      "se3_controller_node" \
      "odom_to_base.py" \
      "odom_to_pose.py" \
      "cloud_relay.py"; do
      kill_matching INT "$pattern"
    done

    sleep 2

    for pattern in \
      "__name:=flight_recorder" \
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
      "cloud_relay.py" \
      "roslaunch livox_ros_driver2" \
      "roslaunch fast_lio mapping_mid360.launch" \
      "roslaunch mavros px4.launch" \
      "roslaunch sim2real_common" \
      "roslaunch sim2real_deployment frame_aliases.launch" \
      "static_transform_publisher.*real_world_"; do
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
      "cloud_relay.py" \
      "roslaunch livox_ros_driver2" \
      "roslaunch fast_lio mapping_mid360.launch" \
      "roslaunch mavros px4.launch" \
      "roslaunch sim2real_common" \
      "roslaunch sim2real_deployment frame_aliases.launch" \
      "static_transform_publisher.*real_world_"; do
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

require_real_runtime_stack() {
  ensure_prereqs
  resolve_ros_ip
  if ! tmux_has_session; then
    echo "[ERROR] Real-flight stack '$SESSION_NAME' is not running. Start it before issuing flight commands." >&2
    return 1
  fi
  if ! docker_container_running; then
    echo "[ERROR] Container '$CONTAINER_NAME' is not running." >&2
    return 1
  fi
}

require_real_control_stack() {
  require_real_runtime_stack || return 1
  if ! docker_exec_shell 'rosparam get /run_id >/dev/null 2>&1'; then
    echo "[ERROR] ROS master is not reachable." >&2
    return 1
  fi
}

validate_real_command_settings() {
  if ! [[ "$REAL_COMMAND_TIMEOUT" =~ ^[1-9][0-9]*$ ]]; then
    echo "[ERROR] REAL_COMMAND_TIMEOUT must be a positive integer." >&2
    return 1
  fi
}

validate_takeoff_settings() {
  if ! [[ "$REAL_TAKEOFF_TIMEOUT" =~ ^[1-9][0-9]*$ ]]; then
    echo "[ERROR] REAL_TAKEOFF_TIMEOUT must be a positive integer." >&2
    return 1
  fi
  if ! python3 -c \
    'import math,sys; height,tolerance,preflight,stable,max_vz=map(float,sys.argv[1:]); raise SystemExit(0 if all(map(math.isfinite,(height,tolerance,preflight,stable,max_vz))) and height > 0.0 and 0.0 < tolerance < height and preflight > 0.0 and stable > 0.0 and max_vz > 0.0 else 1)' \
    "$REAL_TAKEOFF_HEIGHT" "$REAL_TAKEOFF_TOLERANCE" "$REAL_PREFLIGHT_TIMEOUT" \
    "$REAL_TAKEOFF_STABLE_TIME" "$REAL_TAKEOFF_MAX_VERTICAL_SPEED" 2>/dev/null; then
    echo "[ERROR] Real takeoff height, preflight timeout, stable time and max vertical speed must be finite and positive; tolerance must be strictly between zero and the takeoff height." >&2
    return 1
  fi
}

prepare_runtime_planner_config() {
  if [ -z "$PLANNER_RESOLUTION" ] && [ -z "$PLANNER_OBSTACLES_INFLATION" ]; then
    return
  fi
  if [ -n "$PLANNER_CONFIG" ]; then
    echo "[ERROR] PLANNER_CONFIG cannot be combined with PLANNER_RESOLUTION or PLANNER_OBSTACLES_INFLATION." >&2
    return 1
  fi

  local name value
  for name in PLANNER_RESOLUTION PLANNER_OBSTACLES_INFLATION; do
    value="${!name}"
    if [ -n "$value" ] && ! python3 -c \
      'import math,sys; value=float(sys.argv[1]); raise SystemExit(0 if math.isfinite(value) and value > 0.0 else 1)' \
      "$value" 2>/dev/null; then
      echo "[ERROR] $name must be finite and positive." >&2
      return 1
    fi
  done

  local source_config="$PROJECT_ROOT/common/config/planner.yaml"
  local runtime_tmp_dir="$PROJECT_ROOT/runtime/tmp"
  local generated_name="planner_runtime_${RUN_ID}.yaml"
  local generated_host="$runtime_tmp_dir/$generated_name"
  local generated_tmp="${generated_host}.tmp"
  mkdir -p "$runtime_tmp_dir"

  if ! awk \
    -v resolution="$PLANNER_RESOLUTION" \
    -v inflation="$PLANNER_OBSTACLES_INFLATION" '
      resolution != "" && /^  resolution:[[:space:]]*/ {
        print "  resolution: " resolution
        resolution_replaced = 1
        next
      }
      inflation != "" && /^  obstacles_inflation:[[:space:]]*/ {
        print "  obstacles_inflation: " inflation
        inflation_replaced = 1
        next
      }
      { print }
      END {
        if (resolution != "" && !resolution_replaced) exit 10
        if (inflation != "" && !inflation_replaced) exit 11
      }
    ' "$source_config" > "$generated_tmp"; then
    rm -f "$generated_tmp"
    echo "[ERROR] Unable to generate the runtime Planner configuration." >&2
    return 1
  fi
  mv "$generated_tmp" "$generated_host"
  PLANNER_CONFIG="/root/tmp/$generated_name"

  local effective_resolution effective_inflation inflation_summary
  effective_resolution="${PLANNER_RESOLUTION:-$(awk '/^  resolution:/ {print $2; exit}' "$source_config")}"
  effective_inflation="${PLANNER_OBSTACLES_INFLATION:-$(awk '/^  obstacles_inflation:/ {print $2; exit}' "$source_config")}"
  inflation_summary="$(python3 -c \
    'import math,sys; r,i=map(float,sys.argv[1:]); layers=max(0,math.ceil((i-1e-5)/r)); print("{} layer(s), approximately {:.3f} m".format(layers,layers*r))' \
    "$effective_resolution" "$effective_inflation")"
  echo "[INFO] Runtime Planner map override: resolution=$effective_resolution m, obstacles_inflation=$effective_inflation m ($inflation_summary)."
}

vehicle_is_connected() {
  docker_exec_shell \
    "timeout 4 rostopic echo -n 1 /mavros/state 2>/dev/null | grep -q 'connected: True'" >/dev/null 2>&1
}

vehicle_is_armed() {
  docker_exec_shell \
    "timeout 4 rostopic echo -n 1 /mavros/state 2>/dev/null | grep -q 'armed: True'" >/dev/null 2>&1
}

vehicle_is_offboard() {
  docker_exec_shell \
    "timeout 4 rostopic echo -n 1 /mavros/state 2>/dev/null | grep -q 'mode: \"OFFBOARD\"'" >/dev/null 2>&1
}

vehicle_is_auto_land() {
  docker_exec_shell \
    "timeout 4 rostopic echo -n 1 /mavros/state 2>/dev/null | grep -q 'mode: \"AUTO.LAND\"'" >/dev/null 2>&1
}

arm_vehicle() {
  need_cmd python3
  require_real_control_stack || return 1
  if [ ! -f "$ARM_EXECUTOR_HOST" ]; then
    echo "[ERROR] Shared arm executor not found: $ARM_EXECUTOR_HOST" >&2
    return 1
  fi

  echo "[INFO] Starting the shared low-latency arm/takeoff/OFFBOARD state machine..."
  docker_exec_shell \
    "python3 -u - \
      --takeoff-height '$REAL_TAKEOFF_HEIGHT' \
      --preflight-timeout '$REAL_PREFLIGHT_TIMEOUT' \
      --command-timeout '$REAL_COMMAND_TIMEOUT' \
      --takeoff-timeout '$REAL_TAKEOFF_TIMEOUT' \
      --takeoff-tolerance '$REAL_TAKEOFF_TOLERANCE' \
      --takeoff-stable-time '$REAL_TAKEOFF_STABLE_TIME' \
      --takeoff-max-vertical-speed '$REAL_TAKEOFF_MAX_VERTICAL_SPEED' \
      --odometry-topic '$LOCALIZATION_ODOM_TOPIC' \
      --controller-node '$SE3_NODE_NAME' \
      --attitude-setpoint-topic '$MAVROS_ATTITUDE_TOPIC'" \
    < "$ARM_EXECUTOR_HOST"
}

publish_goal() {
  if [ "$#" -lt 3 ] || [ "$#" -gt 4 ]; then
    echo "[ERROR] Usage: $0 goal X Y Z [YAW_DEG]" >&2
    return 1
  fi
  need_cmd python3
  require_real_runtime_stack || return 1
  if [ ! -f "$GOAL_EXECUTOR_HOST" ]; then
    echo "[ERROR] Shared goal executor not found: $GOAL_EXECUTOR_HOST" >&2
    return 1
  fi

  local x="$1" y="$2" z="$3" yaw_spec="${4:-}"
  python3 -c \
    'import math,sys; values=[float(v) for v in sys.argv[1:]]; raise SystemExit(0 if all(math.isfinite(v) for v in values) else 1)' \
    "$@" || {
    echo "[ERROR] Goal X, Y, Z, and optional YAW_DEG must be finite numbers." >&2
    return 1
  }

  local yaw_arg=""
  if [ -n "$yaw_spec" ]; then
    yaw_arg="--yaw-deg '$yaw_spec'"
  fi

  echo "[INFO] Starting the shared low-latency goal validation and publisher..."
  docker_exec_shell \
    "python3 -u - '$x' '$y' '$z' \
      $yaw_arg \
      --drone-id '$DRONE_ID' \
      --preflight-timeout '$REAL_PREFLIGHT_TIMEOUT' \
      --odometry-topic '$LOCALIZATION_ODOM_TOPIC' \
      --controller-node '$SE3_NODE_NAME' \
      --attitude-setpoint-topic '$MAVROS_ATTITUDE_TOPIC'" \
    < "$GOAL_EXECUTOR_HOST"
}

request_land() {
  validate_real_command_settings
  require_real_control_stack

  if ! vehicle_is_connected; then
    echo "[ERROR] MAVROS is not connected to PX4." >&2
    return 1
  fi
  if ! vehicle_is_armed; then
    echo "[INFO] Vehicle is already disarmed; no landing request is needed."
    return 0
  fi

  echo "[WARN] Requesting PX4 AUTO.LAND. Keep the RC ready for immediate takeover." >&2
  local waited=0
  while ! vehicle_is_auto_land; do
    docker_exec_shell \
      "rosservice call /mavros/set_mode \"base_mode: 0
custom_mode: 'AUTO.LAND'\" 2>/dev/null | grep -q 'mode_sent: True'" >/dev/null 2>&1 || true
    if [ "$waited" -ge "$REAL_COMMAND_TIMEOUT" ]; then
      echo "[ERROR] PX4 did not enter AUTO.LAND within ${REAL_COMMAND_TIMEOUT}s; take over with the RC." >&2
      return 1
    fi
    sleep 1
    waited=$((waited + 1))
  done
  echo "[INFO] PX4 AUTO.LAND is active. This command will not force-disarm the vehicle."
}

run_waypoint_mission() {
  if [ "$#" -ne 1 ]; then
    echo "[ERROR] Usage: $0 mission FILE" >&2
    return 1
  fi
  local mission_file="$1"
  need_cmd python3
  validate_real_command_settings
  validate_takeoff_settings
  require_real_control_stack

  if [ ! -f "$mission_file" ]; then
    echo "[ERROR] Mission file not found: $mission_file" >&2
    return 1
  fi
  if [ ! -f "$MISSION_RUNNER_HOST" ]; then
    echo "[ERROR] Mission runner not found: $MISSION_RUNNER_HOST" >&2
    return 1
  fi
  if [ ! -f "$MISSION_EXECUTOR_HOST" ]; then
    echo "[ERROR] Shared mission executor not found: $MISSION_EXECUTOR_HOST" >&2
    return 1
  fi

  # This is the same executor and waypoint runner used by sim.sh. The shell
  # wrapper supplies only the container and timeout values.
  local container_mission_dir="/tmp/sim2real_mission_$$"
  local container_mission="$container_mission_dir/mission_runtime.json"
  local container_runner="$container_mission_dir/waypoint_mission.py"
  local container_executor="$container_mission_dir/mission_executor.py"
  docker exec -i "$CONTAINER_NAME" mkdir -p "$container_mission_dir" >/dev/null || {
    echo "[ERROR] Failed to create the shared mission runtime directory." >&2
    return 1
  }
  docker cp -- "$mission_file" "$CONTAINER_NAME:$container_mission" >/dev/null || {
    echo "[ERROR] Failed to copy the mission file into the real-flight container." >&2
    return 1
  }
  docker cp -- "$MISSION_RUNNER_HOST" "$CONTAINER_NAME:$container_runner" >/dev/null || {
    echo "[ERROR] Failed to copy the shared waypoint runner into the real-flight container." >&2
    return 1
  }
  docker cp -- "$MISSION_EXECUTOR_HOST" "$CONTAINER_NAME:$container_executor" >/dev/null || {
    echo "[ERROR] Failed to copy the shared mission executor into the real-flight container." >&2
    return 1
  }

  local mission_rc=0
  if docker_exec_shell \
    "python3 -u '$container_executor' '$container_mission' \
      --drone-id '$DRONE_ID' \
      --default-takeoff-height '$REAL_TAKEOFF_HEIGHT' \
      --preflight-timeout '$REAL_PREFLIGHT_TIMEOUT' \
      --command-timeout '$REAL_COMMAND_TIMEOUT' \
      --takeoff-timeout '$REAL_TAKEOFF_TIMEOUT' \
      --takeoff-tolerance '$REAL_TAKEOFF_TOLERANCE' \
      --takeoff-stable-time '$REAL_TAKEOFF_STABLE_TIME' \
      --takeoff-max-vertical-speed '$REAL_TAKEOFF_MAX_VERTICAL_SPEED'"; then
    mission_rc=0
  else
    mission_rc=$?
  fi
  docker exec -i "$CONTAINER_NAME" \
    rm -f "$container_mission" "$container_runner" "$container_executor" \
    >/dev/null 2>&1 || true
  docker exec -i "$CONTAINER_NAME" rmdir "$container_mission_dir" \
    >/dev/null 2>&1 || true

  if [ "$mission_rc" -eq 10 ]; then
    echo "[WARN] Shared mission stopped because RC/manual takeover was detected." >&2
    return 10
  fi
  if [ "$mission_rc" -ne 0 ]; then
    echo "[ERROR] Shared mission failed; no wrapper-specific recovery action was added." >&2
    return "$mission_rc"
  fi
  echo "[INFO] Shared mission completed successfully."
}

create_window() {
  local window_name="$1"
  local inner_cmd="$2"

  tmux new-window -t "$SESSION_NAME" -n "$window_name" "$(docker_tmux_cmd "$inner_cmd")"
  enable_window_logging "$window_name"
}

cleanup_failed_start() {
  local status="$1"
  trap - ERR
  echo "[WARN] Real-flight stack startup failed; cleaning up the partial stack." >&2
  set +e
  stop_stack
  set -e
  return "$status"
}

start_stack() {
  ensure_prereqs
  validate_rosbag_settings
  prepare_runtime_planner_config
  resolve_ros_ip
  ensure_container_running

  if tmux_has_session; then
    echo "[ERROR] tmux session '$SESSION_NAME' already exists." >&2
    echo "Use '$0 attach' to view it or '$0 restart' to recreate it." >&2
    exit 1
  fi

  if container_processes_running; then
    echo "[ERROR] Real-flight processes are already running without the owned tmux session." >&2
    echo "Run '$0 stop' to clean the stale partial stack before starting again." >&2
    exit 1
  fi

  echo "[INFO] Creating tmux session: $SESSION_NAME"
  echo "[INFO] ROS_MASTER_URI=$ROS_MASTER_URI"
  echo "[INFO] ROS_IP=$ROS_IP"
  echo "[INFO] Host tmux logs: $HOST_LOG_DIR"
  echo "[INFO] Container ROS logs: $CONTAINER_ROS_LOG_DIR"

  trap 'cleanup_failed_start "$?"' ERR
  tmux new-session -d -s "$SESSION_NAME" -n roscore "$(docker_tmux_cmd "roscore")"
  tmux set-option -t "$SESSION_NAME" remain-on-exit on
  tmux set-option -t "$SESSION_NAME" history-limit 20000
  enable_window_logging "roscore"

  wait_for_condition "ROS master" "rosparam get /run_id >/dev/null"

  # Clean up publishers created by older versions of real_rviz.sh, then make
  # the frame aliases part of the Jetson-owned runtime instead of the UI.
  docker_exec_shell "rosnode kill /jetson_rviz_world_camera_init_tf /jetson_rviz_world_map_tf >/dev/null 2>&1 || true"
  create_window "frame_aliases" \
    "source ~/.bashrc && roslaunch sim2real_deployment frame_aliases.launch"

  wait_for_condition "static frame aliases" \
    "rosnode list | grep -qx '/real_world_camera_init_tf' && rosnode list | grep -qx '/real_world_map_tf'"

  create_window "mid360" \
    "source /opt/ros/noetic/setup.bash && source ~/livox_ws/devel/setup.bash && roslaunch livox_ros_driver2 $LIVOX_LAUNCH"

  wait_for_condition "Mid-360S topics" "rostopic list | grep -q '^/livox/lidar$' && rostopic list | grep -q '^/livox/imu$' && [ \"\$(rostopic type /livox/lidar)\" = 'livox_ros_driver2/CustomMsg' ]"

  create_window "fast_lio" \
    "source ~/.bashrc && roslaunch fast_lio mapping_mid360.launch rviz:=$FASTLIO_RVIZ"

  wait_for_condition "FAST-LIO odometry" "rostopic list | grep -qx '$ODOM_RAW_TOPIC'"

  create_window "odom_to_base" \
    "source ~/.bashrc && rosrun sim2real_deployment odom_to_base.py _input_topic:=$ODOM_RAW_TOPIC _output_topic:=$LOCALIZATION_ODOM_TOPIC _output_frame_id:=world _output_child_frame_id:=base_link _mount_x:=$MOUNT_X _mount_y:=$MOUNT_Y _mount_z:=$MOUNT_Z _mount_roll_deg:=$MOUNT_ROLL_DEG _mount_pitch_deg:=$MOUNT_PITCH_DEG _mount_yaw_deg:=$MOUNT_YAW_DEG"

  wait_for_condition "shared localization odometry" "timeout 5s rostopic echo -n 1 '$LOCALIZATION_ODOM_TOPIC/header' >/dev/null"
  wait_for_condition "world to base_link TF" \
    "timeout 5s rosrun tf tf_echo world base_link 2>&1 | grep -q 'Translation:'"

  create_window "cloud_adapter" \
    "source ~/.bashrc && rosrun sim2real_deployment cloud_relay.py _input_topic:=$RAW_REGISTERED_CLOUD_TOPIC _output_topic:=$LOCALIZATION_CLOUD_TOPIC _frame_id:=world"

  wait_for_condition "shared registered point cloud" "timeout 5s rostopic echo -n 1 '$LOCALIZATION_CLOUD_TOPIC/header' >/dev/null"

  tmux new-window -t "$SESSION_NAME" -n mavros \
    "docker exec -it -e ROS_MASTER_URI='$ROS_MASTER_URI' -e ROS_IP='$ROS_IP' -e ROS_LOG_DIR='$CONTAINER_ROS_LOG_DIR' -e HOST_FCU_URL='$FCU_URL' -e HOST_GCS_URL='$GCS_URL' '$CONTAINER_NAME' bash -lc 'unset ROS_HOSTNAME && source ~/.bashrc && mkdir -p \"\$ROS_LOG_DIR\" && if [ -n \"\$HOST_FCU_URL\" ]; then export FCU_URL=\"\$HOST_FCU_URL\"; fi && if [ -n \"\$HOST_GCS_URL\" ]; then export GCS_URL=\"\$HOST_GCS_URL\"; fi && roslaunch mavros px4.launch fcu_url:=\"\$FCU_URL\" gcs_url:=\"\$GCS_URL\" tgt_system:=$MAVROS_TGT_SYSTEM'"
  enable_window_logging "mavros"

  wait_for_condition "MAVROS connection" "rostopic list | grep -q '^/mavros/state$' && timeout 3s rostopic echo -n 1 /mavros/state | grep -q 'connected: True'"

  create_window "odom_to_pose" \
    "source ~/.bashrc && rosrun sim2real_deployment odom_to_pose.py _odom_topic:=$LOCALIZATION_ODOM_TOPIC _pose_topic:=/mavros/vision_pose/pose _frame_id:=map _publish_rate:=30.0 _max_input_age:=0.2 _use_input_stamp:=false"

  wait_for_condition "vision pose bridge" "rostopic list | grep -q '^/mavros/vision_pose/pose$'"

  # This exact stack-lifetime guard also runs in simulation. A localization
  # outage or impossible odometry value is latched until a full stack restart;
  # an autonomous flight is changed to AUTO.LAND immediately.
  create_window "localization_guard" \
    "source ~/.bashrc && rosrun sim2real_common localization_guard.py _odometry_topic:=$LOCALIZATION_ODOM_TOPIC"

  wait_for_condition "localization safety guard" \
    "rosnode list | grep -qx '/localization_guard'"

  if [ "$START_DIFF_PLANNER" = "true" ]; then
    local planner_cmd
    planner_cmd="source ~/.bashrc && roslaunch sim2real_common planner.launch drone_id:=$DRONE_ID odom_topic:=$LOCALIZATION_ODOM_TOPIC cloud_topic:=$LOCALIZATION_CLOUD_TOPIC"
    if [ -n "$PLANNER_CONFIG" ]; then
      planner_cmd+=" planner_config:=$PLANNER_CONFIG"
    fi
    create_window "diff_planner" \
      "$planner_cmd"

    wait_for_condition "Diff-Planner" \
      "rosnode list | grep -qx '/drone_${DRONE_ID}_diff_planner_node' && rosnode list | grep -qx '/drone_${DRONE_ID}_traj_server' && rostopic list | grep -qx '$DIFF_PLANNER_POS_CMD_TOPIC'"
  else
    echo "[INFO] Diff-Planner startup skipped because START_DIFF_PLANNER=false."
  fi

  if [ "$START_TRAJ_CONVERTER" = "true" ]; then
    create_window "traj_converter" \
      "source ~/.bashrc && roslaunch sim2real_common trajectory_converter.launch drone_id:=$DRONE_ID output_topic:=$TRAJ_CONVERTER_OUTPUT_TOPIC replay_cached_goal_on_offboard:=false"

    wait_for_condition "trajectory converter" "rosnode list | grep -qx '/trajectory_msg_converter' && rostopic list | grep -qx '$TRAJ_CONVERTER_OUTPUT_TOPIC'"
  else
    echo "[INFO] trajectory converter startup skipped because START_TRAJ_CONVERTER=false."
  fi

  if [ "$START_SE3_CONTROLLER" = "true" ]; then
    create_window "se3_controller" \
      "source ~/.bashrc && roslaunch sim2real_common controller.launch vehicle_config:=$CONTROLLER_CONFIG"

    wait_for_condition "SE3 controller" "rosnode list | grep -qx '$SE3_NODE_NAME' && rostopic list | grep -qx '$MAVROS_ATTITUDE_TOPIC'"
  else
    echo "[INFO] SE3 controller startup skipped because START_SE3_CONTROLLER=false."
  fi

  if [ "$START_ROSBAG" = "true" ]; then
    local available_mb min_free_mb rosbag_min_space_arg
    min_free_mb=$((ROSBAG_MIN_FREE_GB * 1024))
    rosbag_min_space_arg=""
    if [ "$ROSBAG_MIN_FREE_GB" -gt 0 ]; then
      rosbag_min_space_arg="--min-space=${ROSBAG_MIN_FREE_GB}G"
    fi
    available_mb="$(docker_exec_shell "mkdir -p '$ROSBAG_DIR' && df -Pm '$ROSBAG_DIR' | awk 'NR == 2 {print \$4}'" | tail -n 1)"
    if ! [[ "$available_mb" =~ ^[0-9]+$ ]]; then
      echo "[ERROR] Unable to determine free space for $ROSBAG_DIR." >&2
      return 1
    fi
    if [ "$ROSBAG_MIN_FREE_GB" -gt 0 ] && [ "$available_mb" -lt "$min_free_mb" ]; then
      echo "[ERROR] Only ${available_mb} MB is free in $ROSBAG_DIR; at least ${min_free_mb} MB is required." >&2
      return 1
    fi

    write_rosbag_state "$ROSBAG_DIR/${ROSBAG_PREFIX}_${RUN_ID}"
    create_window "rosbag" \
      "source ~/.bashrc && mkdir -p '$ROSBAG_DIR' && exec nice -n '$ROSBAG_NICE_LEVEL' rosbag record --lz4 --split --size='$ROSBAG_SPLIT_SIZE_MB' --repeat-latched $rosbag_min_space_arg -O '$ROSBAG_DIR/${ROSBAG_PREFIX}_${RUN_ID}' __name:=flight_recorder $ROSBAG_TOPICS $ROSBAG_EXTRA_ARGS"

    wait_for_condition "rosbag recorder" "rosnode list | grep -qx '$ROSBAG_NODE_NAME'"
    echo "[INFO] Recording split bags with prefix (container) $ROSBAG_DIR/${ROSBAG_PREFIX}_${RUN_ID}"
  else
    echo "[INFO] rosbag recording skipped because START_ROSBAG=false."
  fi

  echo "[INFO] Real-flight stack started successfully."
  echo "[INFO] Use '$0 attach' to inspect tmux windows."
  echo "[INFO] Host tmux logs: $HOST_LOG_DIR"
  echo "[INFO] Container ROS logs: $CONTAINER_ROS_LOG_DIR"
  trap - ERR
}

stop_stack() {
  ensure_prereqs

  echo "[INFO] Stopping real-flight stack..."

  local stopping_rosbag_prefix=""
  if docker_container_exists && docker_container_running; then
    stopping_rosbag_prefix="$(current_rosbag_output_prefix 2>/dev/null || true)"
  fi
  stop_rosbag_gracefully || true

  if tmux_has_session; then
    for window_name in se3_controller traj_converter diff_planner localization_guard odom_to_pose mavros cloud_adapter odom_to_base fast_lio mid360 frame_aliases roscore; do
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
  finalize_indexed_active_bags "$stopping_rosbag_prefix" || true

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
  shift || true

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
    arm)
      arm_vehicle
      ;;
    land)
      request_land
      ;;
    goal)
      publish_goal "$@"
      ;;
    mission)
      run_waypoint_mission "$@"
      ;;
    *)
      usage
      exit 1
      ;;
  esac
}

if [ "${BASH_SOURCE[0]}" = "$0" ]; then
  main "$@"
fi
