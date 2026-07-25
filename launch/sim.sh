#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

SCENE="${SIM_SCENE:-default}"
if [ "${1:-}" = "--scene" ]; then
  [ "$#" -ge 3 ] || {
    echo "[ERROR] Usage: sim.sh --scene NAME ACTION [ARGS...]" >&2
    exit 1
  }
  SCENE="$2"
  shift 2
elif [[ "${1:-}" == --scene=* ]]; then
  SCENE="${1#--scene=}"
  [ -n "$SCENE" ] || {
    echo "[ERROR] --scene requires a non-empty scene name." >&2
    exit 1
  }
  shift
fi

if [[ "$SCENE" == */* ]]; then
  if [[ "$SCENE" = /* ]]; then
    SCENE_CONFIG="$SCENE"
  else
    SCENE_CONFIG="$PROJECT_ROOT/$SCENE"
  fi
else
  SCENE_FILE="$SCENE"
  [[ "$SCENE_FILE" == *.env ]] || SCENE_FILE+=".env"
  SCENE_CONFIG="$PROJECT_ROOT/simulation/config/scenes/$SCENE_FILE"
fi
[ -f "$SCENE_CONFIG" ] || {
  echo "[ERROR] Simulation scene config not found: $SCENE_CONFIG" >&2
  exit 1
}
# Scene profiles only describe the Gazebo world and vehicle spawn pose.
# Explicit SIM_WORLD/SIM_SPAWN_* environment variables always take priority.
# shellcheck disable=SC1090
source "$SCENE_CONFIG"

DEV_CONTAINER="${SIM_DEV_CONTAINER:-diff_planner_px4_sim}"
# PX4/Gazebo/Mid360 and the development overlay now run in the same
# repository-owned container. Keep one name internally so no command can
# silently fall back to the legacy ros_noetic container.
SIMULATOR_CONTAINER="$DEV_CONTAINER"
DEV_CONTAINER_SCRIPT="$SCRIPT_DIR/sim_container.sh"
DEV_WORKSPACE="${SIM_WORKSPACE_CONTAINER:-/workspaces/sim2real_ws}"
DEV_SOURCE="$DEV_WORKSPACE/src/Diff-Planner-PX4"
COMMON_SOURCE="$DEV_WORKSPACE/src/sim2real_common"
ADAPTER_SOURCE="$DEV_WORKSPACE/src/sim2real_simulation"
DEV_RUNTIME="${SIM_RUNTIME_CONTAINER:-/root/simulation_runtime}"
BASE_SIM_WORKSPACE="${SIM_BASE_WORKSPACE_CONTAINER:-/opt/simulation_ws}"
SOURCE_HOST="${SIM_SOURCE_HOST:-$PROJECT_ROOT/third_party/Diff-Planner-PX4}"
SESSION_NAME="${SIM_SESSION_NAME:-diff_planner_sim}"
RUN_ID="${SIM_RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
RUNTIME_HOST="${SIM_RUNTIME_HOST:-$PROJECT_ROOT/runtime/simulation}"
HOST_LOG_DIR="${SIM_HOST_LOG_DIR:-$RUNTIME_HOST/runs/$RUN_ID/tmux}"
SESSION_MARKER="$RUNTIME_HOST/active/${SESSION_NAME}.owner"
DEV_SESSION_MARKER="$DEV_RUNTIME/active/${SESSION_NAME}.owner"
START_LOCK="$RUNTIME_HOST/active/${SESSION_NAME}.start.lock"
START_LOCK_FD=""
START_CREATED_SESSION=false

ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
ROS_IP="${ROS_IP:-127.0.0.1}"
DEV_ROS_HOME="${DEV_ROS_HOME:-$DEV_RUNTIME/runs/$RUN_ID/ros_home}"
DEV_ROS_LOG_DIR="${DEV_ROS_LOG_DIR:-$DEV_RUNTIME/runs/$RUN_ID/ros_logs}"
SIM_ROS_HOME="${SIM_ROS_HOME:-$DEV_ROS_HOME}"
SIM_ROS_LOG_DIR="${SIM_ROS_LOG_DIR:-$DEV_ROS_LOG_DIR}"

START_TIMEOUT="${SIM_START_TIMEOUT:-120}"
WAIT_INTERVAL="${SIM_WAIT_INTERVAL:-2}"
BUILD_JOBS="${SIM_BUILD_JOBS:-4}"
SKIP_BUILD="${SIM_SKIP_BUILD:-false}"
GAZEBO_GUI="${SIM_GAZEBO_GUI:-true}"
WORLD="${SIM_WORLD:-${SCENE_WORLD:-}}"
SPAWN_X="${SIM_SPAWN_X:-${SCENE_SPAWN_X:-2.0}}"
SPAWN_Y="${SIM_SPAWN_Y:-${SCENE_SPAWN_Y:-0.0}}"
SPAWN_Z="${SIM_SPAWN_Z:-${SCENE_SPAWN_Z:-0.0}}"
SPAWN_ROLL="${SIM_SPAWN_ROLL:-${SCENE_SPAWN_ROLL:-0.0}}"
SPAWN_PITCH="${SIM_SPAWN_PITCH:-${SCENE_SPAWN_PITCH:-0.0}}"
SPAWN_YAW="${SIM_SPAWN_YAW:-${SCENE_SPAWN_YAW:-0.0}}"
START_PLANNER="${SIM_START_PLANNER:-true}"
START_SE3="${SIM_START_SE3:-true}"
START_GOAL_BRIDGE="${SIM_START_GOAL_BRIDGE:-true}"
START_RVIZ="${SIM_START_RVIZ:-true}"
CLOUD_FILTER_ENABLE="${SIM_CLOUD_FILTER_ENABLE:-false}"
CLOUD_VOXEL_LEAF_SIZE="${SIM_CLOUD_VOXEL_LEAF_SIZE:-0.08}"
CLOUD_MAX_POINTS="${SIM_CLOUD_MAX_POINTS:-80000}"
CLOUD_MIN_RANGE="${SIM_CLOUD_MIN_RANGE:-0.2}"
CLOUD_MAX_RANGE="${SIM_CLOUD_MAX_RANGE:-50.0}"
RVIZ_GOAL_Z="${SIM_RVIZ_GOAL_Z:-1.0}"
TAKEOFF_HEIGHT="${SIM_TAKEOFF_HEIGHT:-1.0}"
TAKEOFF_TIMEOUT="${SIM_TAKEOFF_TIMEOUT:-30}"
TAKEOFF_TOLERANCE="${SIM_TAKEOFF_TOLERANCE:-0.1}"
TAKEOFF_STABLE_TIME="${SIM_TAKEOFF_STABLE_TIME:-0.5}"
TAKEOFF_MAX_VERTICAL_SPEED="${SIM_TAKEOFF_MAX_VERTICAL_SPEED:-0.2}"
DISARMED_PREARM_MODE="${SIM_DISARMED_PREARM_MODE:-AUTO.LOITER}"
TAKEOFF_ALTITUDE_FIELD="${SIM_TAKEOFF_ALTITUDE_FIELD:-auto}"
PX4_HOVER_THRUST="${SIM_PX4_HOVER_THRUST:-0.755}"
COMMAND_TIMEOUT="${SIM_COMMAND_TIMEOUT:-15}"
DRONE_ID="${SIM_DRONE_ID:-0}"
PLANNER_CONFIG="${SIM_PLANNER_CONFIG:-}"
CONTROLLER_CONFIG="${SIM_CONTROLLER_CONFIG:-/etc/sim2real/simulation/controller.yaml}"
RVIZ_CONFIG="${SIM_RVIZ_CONFIG:-/etc/sim2real/simulation/rviz/sim.rviz}"
REQUIRE_ARMED_GOAL="${SIM_REQUIRE_ARMED_GOAL:-true}"
TEST_PACKAGES="${SIM_TEST_PACKAGES:-diff_planner sim2real_common sim2real_simulation}"
MISSION_RUNNER_HOST="$PROJECT_ROOT/common/scripts/waypoint_mission.py"
MISSION_EXECUTOR_HOST="$PROJECT_ROOT/common/scripts/mission_executor.py"
ARM_EXECUTOR_HOST="$PROJECT_ROOT/common/scripts/arm_executor.py"
GOAL_EXECUTOR_HOST="$PROJECT_ROOT/common/scripts/goal_executor.py"
PREFLIGHT_TIMEOUT="${SIM_PREFLIGHT_TIMEOUT:-5.0}"

usage() {
  cat <<'EOF'
Usage: sim.sh [--scene NAME] {build|test|start|stop|restart|status|attach|arm|land|goal|mission|shell}

Actions:
  build                 Incrementally build the host deployment source.
  test                  Build, then test diff_planner and both adapter packages.
  start                 Build and start the complete simulation stack.
  restart               Stop, rebuild changed code, and start again.
  stop                  Gracefully stop this script's simulation stack.
  status                Show containers, tmux windows, nodes, and source paths.
  attach                Attach to the host tmux session.
  arm                   Arm, use PX4 AUTO.TAKEOFF, then enter OFFBOARD hold.
  land                  Request landing and wait for simulated disarm.
  goal X Y Z [YAW_DEG]
                        Publish a world-frame goal after OFFBOARD + arm.
                        Omit YAW_DEG to leave the final yaw unconstrained.
  mission FILE          Native takeoff if needed, enter OFFBOARD, execute the
                        ordered JSON waypoints, then automatically land.
                        A mode change away from OFFBOARD aborts the mission.
  shell                 Open a shell in the repository-owned simulation container.

Useful overrides:
  SIM_GAZEBO_GUI=false SIM_START_RVIZ=false ./launch/sim.sh restart
  ./launch/sim.sh --scene se3_test_20260723_151241_0 restart
  SIM_WORLD=/root/simulation_runtime/reconstructed/test/world.world \
    SIM_SPAWN_X=0 SIM_SPAWN_Y=0 ./launch/sim.sh restart
  ./launch/sim.sh arm
  ./launch/sim.sh goal 1.0 0.0 1.0
  ./launch/sim.sh goal 1.0 0.0 1.0 0
  SIM_SKIP_BUILD=true ./launch/sim.sh start
EOF
}

info() {
  echo "[INFO] $*"
}

warn() {
  echo "[WARN] $*" >&2
}

die() {
  echo "[ERROR] $*" >&2
  exit 1
}

require_bool() {
  local name="$1" value="$2"
  case "$value" in
    true|false) ;;
    *) die "$name must be exactly 'true' or 'false' (got: $value)." ;;
  esac
}

require_number() {
  local name="$1" value="$2"
  [[ "$value" =~ ^[-+]?([0-9]+([.][0-9]*)?|[.][0-9]+)([eE][-+]?[0-9]+)?$ ]] ||
    die "$name must be a finite number (got: $value)."
}

validate_bool_config() {
  require_bool SIM_SKIP_BUILD "$SKIP_BUILD"
  require_bool SIM_GAZEBO_GUI "$GAZEBO_GUI"
  require_bool SIM_START_PLANNER "$START_PLANNER"
  require_bool SIM_START_SE3 "$START_SE3"
  require_bool SIM_START_GOAL_BRIDGE "$START_GOAL_BRIDGE"
  require_bool SIM_START_RVIZ "$START_RVIZ"
  require_bool SIM_CLOUD_FILTER_ENABLE "$CLOUD_FILTER_ENABLE"
  require_bool SIM_REQUIRE_ARMED_GOAL "$REQUIRE_ARMED_GOAL"
  require_number SIM_SPAWN_X "$SPAWN_X"
  require_number SIM_SPAWN_Y "$SPAWN_Y"
  require_number SIM_SPAWN_Z "$SPAWN_Z"
  require_number SIM_SPAWN_ROLL "$SPAWN_ROLL"
  require_number SIM_SPAWN_PITCH "$SPAWN_PITCH"
  require_number SIM_SPAWN_YAW "$SPAWN_YAW"
  require_number SIM_CLOUD_VOXEL_LEAF_SIZE "$CLOUD_VOXEL_LEAF_SIZE"
  require_number SIM_CLOUD_MIN_RANGE "$CLOUD_MIN_RANGE"
  require_number SIM_CLOUD_MAX_RANGE "$CLOUD_MAX_RANGE"
  require_number SIM_PX4_HOVER_THRUST "$PX4_HOVER_THRUST"
  [[ "$CLOUD_MAX_POINTS" =~ ^[1-9][0-9]*$ ]] ||
    die "SIM_CLOUD_MAX_POINTS must be a positive integer (got: $CLOUD_MAX_POINTS)."
  case "$DISARMED_PREARM_MODE" in
    STABILIZED|AUTO.LOITER) ;;
    *) die "SIM_DISARMED_PREARM_MODE must be STABILIZED or AUTO.LOITER (got: $DISARMED_PREARM_MODE)." ;;
  esac
  case "$TAKEOFF_ALTITUDE_FIELD" in
    relative|local|auto) ;;
    *) die "SIM_TAKEOFF_ALTITUDE_FIELD must be relative, local, or auto (got: $TAKEOFF_ALTITUDE_FIELD)." ;;
  esac
}

need_cmd() {
  command -v "$1" >/dev/null 2>&1 || die "Missing command: $1"
}

container_exists() {
  docker inspect "$1" >/dev/null 2>&1
}

container_running() {
  [ "$(docker inspect -f '{{.State.Running}}' "$1" 2>/dev/null || echo false)" = "true" ]
}

ensure_prereqs() {
  need_cmd docker
  need_cmd tmux
  need_cmd timeout
}

acquire_start_lock() {
  need_cmd flock
  mkdir -p "$(dirname "$START_LOCK")" || die "Cannot create the simulation lock directory."
  exec {START_LOCK_FD}>"$START_LOCK" || die "Cannot open the simulation start lock: $START_LOCK"
  flock -n "$START_LOCK_FD" ||
    die "Another simulation start/restart is already in progress for session '$SESSION_NAME'."
}

ensure_simulator_container() {
  ensure_dev_container
}

ensure_dev_container() {
  "$DEV_CONTAINER_SCRIPT" run
  docker exec -i \
    -e "SIM_SHARED_RUNTIME=$DEV_RUNTIME" \
    "$DEV_CONTAINER" bash -c '
      mkdir -p "$SIM_SHARED_RUNTIME/runs" "$SIM_SHARED_RUNTIME/active"
      chmod 0777 "$SIM_SHARED_RUNTIME/runs" "$SIM_SHARED_RUNTIME/active"
    '
}

require_running_dev_container() {
  container_running "$DEV_CONTAINER" ||
    die "Simulation container '$DEV_CONTAINER' is not running. Start the simulation first."
}

tmux_has_session() {
  tmux has-session -t "$SESSION_NAME" >/dev/null 2>&1
}

tmux_has_window() {
  tmux list-windows -t "$SESSION_NAME" -F '#W' 2>/dev/null | grep -qx "$1"
}

container_ros_home() {
  if [ "$1" = "$SIMULATOR_CONTAINER" ]; then
    echo "$SIM_ROS_HOME"
  else
    echo "$DEV_ROS_HOME"
  fi
}

container_ros_log_dir() {
  if [ "$1" = "$SIMULATOR_CONTAINER" ]; then
    echo "$SIM_ROS_LOG_DIR"
  else
    echo "$DEV_ROS_LOG_DIR"
  fi
}

container_setup() {
  # root.bashrc sources the optional deployment overlay first, then restores the
  # PX4/Gazebo paths. Sourcing the overlay a second time here would erase those
  # simulator paths from ROS_PACKAGE_PATH and GAZEBO_*_PATH.
  echo "source /root/.bashrc"
}

ros_exec() {
  local container="$1"
  local inner_cmd="$2"
  local ros_home ros_log_dir setup
  ros_home="$(container_ros_home "$container")"
  ros_log_dir="$(container_ros_log_dir "$container")"
  setup="$(container_setup "$container")"

  docker exec -i \
    -e "ROS_MASTER_URI=$ROS_MASTER_URI" \
    -e "ROS_IP=$ROS_IP" \
    -e "ROS_HOME=$ros_home" \
    -e "ROS_LOG_DIR=$ros_log_dir" \
    -e PYTHONDONTWRITEBYTECODE=1 \
    "$container" bash -lc \
    "set -eo pipefail; unset ROS_HOSTNAME; $setup; export ROS_MASTER_URI='$ROS_MASTER_URI' ROS_IP='$ROS_IP' ROS_HOME='$ros_home' ROS_LOG_DIR='$ros_log_dir'; mkdir -p \"\$ROS_HOME\" \"\$ROS_LOG_DIR\"; $inner_cmd"
}

docker_tmux_command() {
  local container="$1"
  local inner_cmd="$2"
  local ros_home ros_log_dir setup shell_cmd
  ros_home="$(container_ros_home "$container")"
  ros_log_dir="$(container_ros_log_dir "$container")"
  setup="$(container_setup "$container")"
  shell_cmd="set -eo pipefail; unset ROS_HOSTNAME; $setup; export ROS_MASTER_URI='$ROS_MASTER_URI' ROS_IP='$ROS_IP' ROS_HOME='$ros_home' ROS_LOG_DIR='$ros_log_dir'; mkdir -p \"\$ROS_HOME\" \"\$ROS_LOG_DIR\"; $inner_cmd"

  printf 'docker exec -it -e ROS_MASTER_URI=%q -e ROS_IP=%q -e ROS_HOME=%q -e ROS_LOG_DIR=%q -e PYTHONDONTWRITEBYTECODE=1 %q bash -lc %q' \
    "$ROS_MASTER_URI" "$ROS_IP" "$ros_home" "$ros_log_dir" "$container" "$shell_cmd"
}

enable_window_logging() {
  local window_name="$1"
  mkdir -p "$HOST_LOG_DIR"
  tmux pipe-pane -t "$SESSION_NAME:$window_name" -o "cat >> '$HOST_LOG_DIR/$window_name.tmux.log'"
}

create_window() {
  local window_name="$1"
  local container="$2"
  local inner_cmd="$3"
  tmux new-window -t "$SESSION_NAME" -n "$window_name" "$(docker_tmux_command "$container" "$inner_cmd")"
  enable_window_logging "$window_name"
}

wait_for_condition() {
  local stage="$1"
  local container="$2"
  local check_cmd="$3"
  local window_name="${4:-}"
  local waited=0

  info "Waiting for $stage ..."
  while ! ros_exec "$container" "$check_cmd" >/dev/null 2>&1; do
    if [ -n "$window_name" ]; then
      if ! tmux_has_window "$window_name" || \
        [ "$(tmux display-message -p -t "$SESSION_NAME:$window_name" '#{pane_dead}' 2>/dev/null || echo 1)" = "1" ]; then
        echo "[ERROR] tmux window '$window_name' exited while waiting for $stage." >&2
        if [ -f "$HOST_LOG_DIR/$window_name.tmux.log" ]; then
          tail -n 40 "$HOST_LOG_DIR/$window_name.tmux.log" >&2 || true
        fi
        return 1
      fi
    fi
    if [ "$waited" -ge "$START_TIMEOUT" ]; then
      echo "[ERROR] Timed out while waiting for $stage." >&2
      echo "[ERROR] Inspect with: $0 attach" >&2
      echo "[ERROR] Host logs: $HOST_LOG_DIR" >&2
      return 1
    fi
    sleep "$WAIT_INTERVAL"
    waited=$((waited + WAIT_INTERVAL))
  done
  info "$stage is ready."
}

build_overlay() {
  ensure_prereqs
  ensure_dev_container

  local build_args=""
  if [ -n "$BUILD_JOBS" ]; then
    [[ "$BUILD_JOBS" =~ ^[1-9][0-9]*$ ]] || die "SIM_BUILD_JOBS must be a positive integer."
    build_args="-j$BUILD_JOBS -p$BUILD_JOBS"
  fi

  info "Incrementally building host source in $DEV_WORKSPACE"
  docker exec -i \
    -e SIM_WORKSPACE_CONTAINER=/__build_from_image_underlay__ \
    -e "SIM_DEV_WORKSPACE=$DEV_WORKSPACE" \
    -e "SIM_BASE_SIM_WORKSPACE=$BASE_SIM_WORKSPACE" \
    -e "SIM_DEV_SOURCE=$DEV_SOURCE" \
    -e "SIM_COMMON_SOURCE=$COMMON_SOURCE" \
    -e "SIM_ADAPTER_SOURCE=$ADAPTER_SOURCE" \
    -e "SIM_CATKIN_BUILD_ARGS=$build_args" \
    -e PYTHONDONTWRITEBYTECODE=1 \
    "$DEV_CONTAINER" bash -s <<'CONTAINER_BUILD_SCRIPT'
set -eo pipefail
source /root/.bashrc
mkdir -p "$SIM_DEV_WORKSPACE/src"
cd "$SIM_DEV_WORKSPACE"

underlay_marker="$SIM_DEV_WORKSPACE/.simulation_underlay"
migrated_underlay=false
if [ ! -f "$underlay_marker" ] || \
   ! grep -qx "schema=sim2real-v1" "$underlay_marker" || \
   ! grep -qx "underlay=$SIM_BASE_SIM_WORKSPACE/devel" "$underlay_marker"; then
  migrated_underlay=true
fi

for profile_file in \
  "$SIM_DEV_WORKSPACE/.catkin_tools/profiles/default/config.yaml" \
  "$SIM_DEV_WORKSPACE/.catkin_tools/profiles/default/build.yaml"; do
  if [ -f "$profile_file" ] && \
     ! grep -qx "extend_path: $SIM_BASE_SIM_WORKSPACE/devel" "$profile_file"; then
    sed -i -E "s#^extend_path:.*#extend_path: $SIM_BASE_SIM_WORKSPACE/devel#" "$profile_file"
    migrated_underlay=true
  fi
done

catkin init
catkin config --extend "$SIM_BASE_SIM_WORKSPACE/devel" --cmake-args -DCMAKE_BUILD_TYPE=Release
if [ "$migrated_underlay" = true ]; then
  echo "[INFO] Migrating catkin underlay to $SIM_BASE_SIM_WORKSPACE/devel; cleaning generated workspace products once."
  catkin clean --yes
fi

read -r -a build_args <<< "$SIM_CATKIN_BUILD_ARGS"
catkin build --no-status "${build_args[@]}"

source "$SIM_DEV_WORKSPACE/devel/setup.bash"
for package in diff_planner se3_controller traj_utils rviz_plugins sim2real_common sim2real_simulation; do
  resolved="$(rospack find "$package")"
  case "$package:$resolved" in
    diff_planner:"$SIM_DEV_SOURCE"/*|se3_controller:"$SIM_DEV_SOURCE"/*|traj_utils:"$SIM_DEV_SOURCE"/*|rviz_plugins:"$SIM_DEV_SOURCE"/*) ;;
    sim2real_common:"$SIM_COMMON_SOURCE"|sim2real_simulation:"$SIM_ADAPTER_SOURCE") ;;
    *) echo "[ERROR] Package $package resolved to stale source: $resolved" >&2; exit 1 ;;
  esac
done
printf 'schema=sim2real-v1\nunderlay=%s\n' "$SIM_BASE_SIM_WORKSPACE/devel" > "$underlay_marker"
CONTAINER_BUILD_SCRIPT
  info "Build complete; common, simulation adapter, and planner sources are active."
}

test_overlay() {
  build_overlay
  # shellcheck disable=SC2206
  local packages=( $TEST_PACKAGES )
  info "Running catkin tests: ${packages[*]}"
  docker exec -i \
    -e "ROS_HOME=$DEV_ROS_HOME" \
    -e "ROS_LOG_DIR=$DEV_ROS_LOG_DIR" \
    "$DEV_CONTAINER" bash -lc "
    set -eo pipefail
    source /root/.bashrc
    export ROS_HOME='$DEV_ROS_HOME' ROS_LOG_DIR='$DEV_ROS_LOG_DIR'
    mkdir -p \"\$ROS_HOME\" \"\$ROS_LOG_DIR\"
    cd '$DEV_WORKSPACE'
    catkin test --no-status ${packages[*]}
    if command -v catkin_test_results >/dev/null 2>&1; then
      catkin_test_results --verbose '$DEV_WORKSPACE/build'
    fi
  "

  info "Validating shared Planner, converter, controller, and simulation adapter launches."
  docker exec -i \
    -e "ROS_MASTER_URI=$ROS_MASTER_URI" \
    -e "ROS_IP=$ROS_IP" \
    "$DEV_CONTAINER" bash -lc "
      set -eo pipefail
      unset ROS_HOSTNAME
      source /root/.bashrc
      export ROS_MASTER_URI='$ROS_MASTER_URI' ROS_IP='$ROS_IP'
      roslaunch --nodes sim2real_common planner.launch
      roslaunch --nodes sim2real_common trajectory_converter.launch
      roslaunch --nodes sim2real_common controller.launch vehicle_config:='$CONTROLLER_CONFIG'
      roslaunch --nodes sim2real_simulation localization.launch
      roslaunch --nodes sim2real_simulation goal_bridge.launch
    "

  ensure_simulator_container
  info "Validating the repository-owned PX4/Gazebo/Mid360 launch expansion."
  docker exec -i \
    -e "ROS_MASTER_URI=$ROS_MASTER_URI" \
    -e "ROS_IP=$ROS_IP" \
    "$SIMULATOR_CONTAINER" bash -lc "
      set -eo pipefail
      unset ROS_HOSTNAME
      source /root/.bashrc
      export ROS_MASTER_URI='$ROS_MASTER_URI' ROS_IP='$ROS_IP'
      roslaunch --nodes px4 outdoor_mid360.launch gui:=false
    "
}

master_is_running() {
  ros_exec "$DEV_CONTAINER" 'rosparam get /run_id >/dev/null' >/dev/null 2>&1
}

create_session_marker() {
  if mkdir -p "$(dirname "$SESSION_MARKER")" 2>/dev/null && \
    printf 'run_id=%s\nsession=%s\n' "$RUN_ID" "$SESSION_NAME" > "$SESSION_MARKER" 2>/dev/null; then
    return 0
  fi

  warn "Host cannot write the simulation marker directly; writing it through the development container."
  docker exec -i \
    -e "SIM_MARKER_PATH=$DEV_SESSION_MARKER" \
    -e "SIM_MARKER_RUN_ID=$RUN_ID" \
    -e "SIM_MARKER_SESSION=$SESSION_NAME" \
    "$DEV_CONTAINER" bash -c '
      mkdir -p "$(dirname "$SIM_MARKER_PATH")"
      printf "run_id=%s\nsession=%s\n" "$SIM_MARKER_RUN_ID" "$SIM_MARKER_SESSION" > "$SIM_MARKER_PATH"
    '
}

remove_session_marker() {
  if rm -f "$SESSION_MARKER" 2>/dev/null; then
    return 0
  fi

  warn "Host cannot remove the simulation marker directly; removing it through the development container."
  docker exec -i \
    -e "SIM_MARKER_PATH=$DEV_SESSION_MARKER" \
    "$DEV_CONTAINER" bash -c 'rm -f -- "$SIM_MARKER_PATH"'
}

cleanup_failed_start() {
  local status="${1:-1}"
  trap - ERR EXIT INT TERM
  if [ "$START_CREATED_SESSION" = "true" ]; then
    warn "Simulation startup failed; cleaning up the partial stack."
    set +e
    stop_stack
    set -e
  fi
  exit "$status"
}

start_stack() {
  ensure_prereqs
  acquire_start_lock
  if tmux_has_session; then
    die "Simulation session '$SESSION_NAME' already exists. Use '$0 attach' or '$0 restart'."
  fi
  if [ -f "$SESSION_MARKER" ]; then
    die "A stale simulation ownership marker exists: $SESSION_MARKER. Run '$0 stop' before starting again."
  fi
  if container_running "$DEV_CONTAINER" && master_is_running; then
    die "A ROS master is already running on 127.0.0.1:11311 outside session '$SESSION_NAME'. Stop the manual stack first."
  fi

  ensure_simulator_container
  if [ -n "$WORLD" ]; then
    ros_exec "$SIMULATOR_CONTAINER" "test -f '$WORLD'" >/dev/null ||
      die "SIM_WORLD does not exist inside '$SIMULATOR_CONTAINER': $WORLD"
  fi

  # ROS commands in the container and tmux logging on the host share this
  # per-run directory. Make it writable across Docker user-namespace mapping.
  docker exec -i \
    -e "SIM_SHARED_RUN_DIR=$DEV_RUNTIME/runs/$RUN_ID" \
    "$DEV_CONTAINER" bash -c '
      mkdir -p "$SIM_SHARED_RUN_DIR"
      chmod 0777 "$SIM_SHARED_RUN_DIR"
    '
  mkdir -p "$HOST_LOG_DIR"

  if [ "$SKIP_BUILD" != "true" ]; then
    build_overlay
  elif ! docker exec -i "$DEV_CONTAINER" test -f "$DEV_WORKSPACE/devel/setup.bash"; then
    die "SIM_SKIP_BUILD=true was requested, but the overlay has not been built yet."
  fi

  info "Starting simulation tmux session: $SESSION_NAME"
  info "Simulation image/container: $SIMULATOR_CONTAINER"
  info "Simulation scene: ${SCENE_DESCRIPTION:-$SCENE}"
  if [ -n "$WORLD" ]; then
    info "Gazebo world: $WORLD"
  fi
  info "Vehicle spawn pose: x=$SPAWN_X y=$SPAWN_Y z=$SPAWN_Z roll=$SPAWN_ROLL pitch=$SPAWN_PITCH yaw=$SPAWN_YAW"
  info "Host source: $SOURCE_HOST"
  info "Host logs: $HOST_LOG_DIR"
  info "SE3 and the RViz goal bridge never auto-arm; '$0 arm' and '$0 mission FILE' are the simulated arming entrypoints."

  trap 'cleanup_failed_start "$?"' EXIT
  trap 'exit 130' INT
  trap 'exit 143' TERM
  tmux new-session -d -s "$SESSION_NAME" -n roscore \
    "$(docker_tmux_command "$SIMULATOR_CONTAINER" 'exec roscore')"
  START_CREATED_SESSION=true
  create_session_marker
  tmux set-option -t "$SESSION_NAME" remain-on-exit on
  tmux set-option -t "$SESSION_NAME" history-limit 30000
  enable_window_logging roscore

  wait_for_condition "ROS master" "$DEV_CONTAINER" 'rosparam get /run_id >/dev/null' roscore

  local sitl_cmd world_arg
  printf -v sitl_cmd \
    'exec roslaunch px4 outdoor_mid360.launch gui:=%q x:=%q y:=%q z:=%q R:=%q P:=%q Y:=%q' \
    "$GAZEBO_GUI" "$SPAWN_X" "$SPAWN_Y" "$SPAWN_Z" \
    "$SPAWN_ROLL" "$SPAWN_PITCH" "$SPAWN_YAW"
  if [ -n "$WORLD" ]; then
    printf -v world_arg ' world:=%q' "$WORLD"
    sitl_cmd+="$world_arg"
  fi
  create_window sitl "$SIMULATOR_CONTAINER" "$sitl_cmd"

  wait_for_condition "Gazebo clock and Iris model" "$SIMULATOR_CONTAINER" \
    "timeout 5 rostopic echo -n 1 /clock >/dev/null && rosservice call /gazebo/get_model_state \"model_name: 'iris'\" 2>/dev/null | grep -q 'success: True'" sitl
  wait_for_condition "MAVROS connection and odometry" "$DEV_CONTAINER" \
    "timeout 5 rostopic echo -n 1 /mavros/state | grep -q 'connected: True' && timeout 5 rostopic echo -n 1 /mavros/local_position/odom/header >/dev/null" sitl
  wait_for_condition "Mid360 PointCloud2" "$DEV_CONTAINER" \
    "[ \"\$(rostopic type /livox/lidar 2>/dev/null)\" = 'sensor_msgs/PointCloud2' ] && width=\"\$(timeout 8 rostopic echo -n 1 /livox/lidar/width 2>/dev/null | awk 'NF && \$1 != \"---\" {print \$1; exit}')\" && [[ \"\$width\" =~ ^[0-9]+\$ ]] && (( width > 0 ))" sitl

  create_window localization "$DEV_CONTAINER" \
    "exec roslaunch sim2real_simulation localization.launch cloud_filter_enable:=$CLOUD_FILTER_ENABLE cloud_voxel_leaf_size:=$CLOUD_VOXEL_LEAF_SIZE cloud_max_points:=$CLOUD_MAX_POINTS cloud_min_range:=$CLOUD_MIN_RANGE cloud_max_range:=$CLOUD_MAX_RANGE"
  wait_for_condition "shared localization odometry" "$DEV_CONTAINER" \
    'timeout 8 rostopic echo -n 1 /localization/odom/header >/dev/null' localization
  wait_for_condition "shared registered point cloud" "$DEV_CONTAINER" \
    'timeout 8 rostopic echo -n 1 /localization/cloud_registered/header >/dev/null' localization

  # Keep the localization failure behavior identical to the real stack.
  create_window localization_guard "$DEV_CONTAINER" \
    'exec rosrun sim2real_common localization_guard.py _odometry_topic:=/localization/odom'
  wait_for_condition "localization safety guard" "$DEV_CONTAINER" \
    "rosnode list | grep -qx '/localization_guard'" localization_guard

  if [ "$START_PLANNER" = "true" ]; then
    local planner_cmd
    planner_cmd='exec roslaunch sim2real_common planner.launch odom_topic:=/localization/odom cloud_topic:=/localization/cloud_registered'
    if [ -n "$PLANNER_CONFIG" ]; then
      planner_cmd+=" planner_config:=$PLANNER_CONFIG"
    fi
    create_window planner "$DEV_CONTAINER" "$planner_cmd"
    wait_for_condition "Diff-Planner nodes" "$DEV_CONTAINER" \
      "rosnode list | grep -q 'diff_planner_node' && rosnode list | grep -q 'traj_server'" planner
    ros_exec "$DEV_CONTAINER" \
      "ground=\$(rosparam get /drone_0_diff_planner_node/grid_map/virtual_ground); ceil=\$(rosparam get /drone_0_diff_planner_node/grid_map/virtual_ceil); awk -v z='$RVIZ_GOAL_Z' -v ground=\"\$ground\" -v ceil=\"\$ceil\" 'BEGIN { exit !(z > ground && z < ceil) }'" >/dev/null || \
      die "SIM_RVIZ_GOAL_Z=$RVIZ_GOAL_Z must be strictly between the shared planner virtual ground and ceiling."

    create_window traj_converter "$DEV_CONTAINER" \
      'exec roslaunch sim2real_common trajectory_converter.launch'
    wait_for_condition "shared trajectory converter" "$DEV_CONTAINER" \
      "rosnode list | grep -qx '/trajectory_msg_converter' && rostopic list | grep -qx '/command/trajectory'" traj_converter
  else
    info "Planner startup skipped (SIM_START_PLANNER=false)."
  fi

  if [ "$START_SE3" = "true" ]; then
    create_window se3 "$DEV_CONTAINER" \
      "exec roslaunch sim2real_common controller.launch vehicle_config:=$CONTROLLER_CONFIG"

    wait_for_condition "fresh-odometry SE3 setpoint stream" "$DEV_CONTAINER" \
      "rosnode list | grep -qx '/se3_controller_node' && timeout 8 rostopic echo -n 1 /mavros/setpoint_position/local/header >/dev/null" se3
  else
    info "SE3 startup skipped (SIM_START_SE3=false)."
  fi

  if [ "$START_GOAL_BRIDGE" = "true" ]; then
    create_window goal_bridge "$DEV_CONTAINER" \
      "exec roslaunch sim2real_simulation goal_bridge.launch goal_z:=$RVIZ_GOAL_Z frame_id:=world"
    wait_for_condition "RViz 2D goal bridge" "$DEV_CONTAINER" \
      "rosnode list | grep -qx '/rviz_2d_goal_bridge' && rosnode info /rviz_2d_goal_bridge | grep -q '/sim2real/rviz_goal'" goal_bridge
    info "RViz 2D goals use z=$RVIZ_GOAL_Z m and require the native-takeoff/automatic-OFFBOARD arm sequence to be complete."
  else
    info "RViz 2D goal bridge skipped (SIM_START_GOAL_BRIDGE=false)."
  fi

  if [ "$START_RVIZ" = "true" ]; then
    ros_exec "$DEV_CONTAINER" "test -f '$RVIZ_CONFIG'" >/dev/null ||
      die "Simulation RViz config not found in the container: $RVIZ_CONFIG"
    create_window rviz "$DEV_CONTAINER" \
      "exec rviz -d '$RVIZ_CONFIG'"
  else
    info "RViz startup skipped (SIM_START_RVIZ=false)."
  fi

  info "Simulation stack is ready."
  info "Inspect logs: $0 attach"
  info "The vehicle is safe/disarmed by default. Use '$0 arm' before publishing a flight goal."
  trap - ERR EXIT INT TERM
  START_CREATED_SESSION=false
}

kill_matching() {
  local container="$1"
  local signal="$2"
  shift 2

  docker exec -i "$container" bash -s -- "$signal" "$@" >/dev/null 2>&1 <<'BASH' || true
signal="$1"
shift
for pattern in "$@"; do
  while IFS= read -r pid; do
    [ -n "$pid" ] || continue
    [ "$pid" = "$$" ] && continue
    [ "$pid" = "$PPID" ] && continue
    kill "-$signal" "$pid" 2>/dev/null || true
  done < <(pgrep -f -- "$pattern" || true)
done
BASH
}

vehicle_is_armed() {
  ros_exec "$DEV_CONTAINER" \
    "timeout 4 rostopic echo -n 1 /mavros/state | grep -q 'armed: True'" >/dev/null 2>&1
}

vehicle_is_offboard() {
  ros_exec "$DEV_CONTAINER" \
    "timeout 4 rostopic echo -n 1 /mavros/state | grep -q 'mode: \"OFFBOARD\"'" >/dev/null 2>&1
}

vehicle_is_connected() {
  ros_exec "$DEV_CONTAINER" \
    "timeout 4 rostopic echo -n 1 /mavros/state | grep -q 'connected: True'" >/dev/null 2>&1
}

request_land() {
  ensure_prereqs
  if ! container_running "$DEV_CONTAINER"; then
    warn "Simulation container is not running; landing request skipped."
    return 0
  fi
  if ! master_is_running; then
    warn "ROS master is not running; landing request skipped."
    return 0
  fi
  if ! vehicle_is_armed; then
    info "Simulated vehicle is already disarmed."
    return 0
  fi

  info "Requesting simulated landing..."
  if ! ros_exec "$DEV_CONTAINER" "rosservice call /land '{}' >/dev/null 2>&1"; then
    warn "/land service failed; falling back to PX4 AUTO.LAND."
    ros_exec "$DEV_CONTAINER" \
      "rosservice call /mavros/set_mode \"base_mode: 0
custom_mode: 'AUTO.LAND'\" >/dev/null"
  fi

  local waited=0
  while vehicle_is_armed; do
    if [ "$waited" -ge 30 ]; then
      warn "Vehicle is still armed after 30 seconds; continuing without forced disarm."
      return 1
    fi
    sleep 2
    waited=$((waited + 2))
  done
  info "Simulated vehicle landed and disarmed."
}

stop_stack() {
  ensure_prereqs
  local owned_stack=false
  local has_session=false

  if tmux_has_session; then
    owned_stack=true
    has_session=true
  elif [ -f "$SESSION_MARKER" ]; then
    owned_stack=true
    warn "tmux session is gone, but an owned simulation marker remains; cleaning orphaned processes."
  else
    info "Simulation tmux session is not running: $SESSION_NAME"
  fi

  if [ "$owned_stack" = "true" ]; then
    request_land || true
  fi

  if [ "$has_session" = "true" ]; then
    info "Stopping simulation session: $SESSION_NAME"
    for window_name in rviz goal_bridge se3 traj_converter planner localization_guard localization sitl roscore; do
      if tmux_has_window "$window_name"; then
        tmux send-keys -t "$SESSION_NAME:$window_name" C-c
      fi
    done
    sleep 5
    tmux kill-session -t "$SESSION_NAME" >/dev/null 2>&1 || true
  fi

  if [ "$owned_stack" = "true" ]; then
    kill_matching "$DEV_CONTAINER" INT \
      "se3_controller_node" "localization_guard.py" "roslaunch sim2real_common" "roslaunch sim2real_simulation" "pointcloud_to_world.py" "sim_odometry_adapter.py" \
      "traj_server" "diff_planner_node" "trajectory_msg_converter.py" "rviz_2d_goal_bridge.py" "rviz"
    kill_matching "$SIMULATOR_CONTAINER" INT \
      "outdoor_mid360.launch" "mavros_node" "gzserver" "gzclient" "PX4-Autopilot.*px4" "rosmaster" "roscore"
    sleep 2
    kill_matching "$DEV_CONTAINER" TERM \
      "se3_controller_node" "localization_guard.py" "roslaunch sim2real_common" "roslaunch sim2real_simulation" "pointcloud_to_world.py" "sim_odometry_adapter.py" \
      "traj_server" "diff_planner_node" "trajectory_msg_converter.py" "rviz_2d_goal_bridge.py" "rviz"
    kill_matching "$SIMULATOR_CONTAINER" TERM \
      "outdoor_mid360.launch" "mavros_node" "gzserver" "gzclient" "PX4-Autopilot.*px4" "rosmaster" "roscore"
    sleep 2
    kill_matching "$DEV_CONTAINER" KILL \
      "se3_controller_node" "localization_guard.py" "roslaunch sim2real_common" "roslaunch sim2real_simulation" "pointcloud_to_world.py" "sim_odometry_adapter.py" \
      "traj_server" "diff_planner_node" "trajectory_msg_converter.py" "rviz_2d_goal_bridge.py" "rviz"
    kill_matching "$SIMULATOR_CONTAINER" KILL \
      "outdoor_mid360.launch" "mavros_node" "gzserver" "gzclient" "PX4-Autopilot.*px4" "rosmaster" "roscore"
    remove_session_marker
  fi
  info "Simulation processes stopped; containers were left running for fast reuse."
}

status_stack() {
  ensure_prereqs
  info "Containers:"
  docker ps -a \
    --filter "name=^/${SIMULATOR_CONTAINER}$" \
    --filter "name=^/${DEV_CONTAINER}$" \
    --format 'table {{.Names}}\t{{.Image}}\t{{.Status}}'

  if tmux_has_session; then
    info "tmux session '$SESSION_NAME' windows:"
    tmux list-windows -t "$SESSION_NAME" -F '#I:#W pane=#{pane_dead} command=#{pane_current_command}'
  else
    info "tmux session is not running: $SESSION_NAME"
  fi

  if container_running "$DEV_CONTAINER" && master_is_running; then
    info "Active shared and simulation package paths:"
    ros_exec "$DEV_CONTAINER" \
      "for package in diff_planner se3_controller traj_utils rviz_plugins sim2real_common sim2real_simulation; do printf '%s=' \"\$package\"; rospack find \"\$package\"; done"
    info "Core ROS nodes:"
    ros_exec "$DEV_CONTAINER" \
      "rosnode list | grep -E 'sitl|gazebo|mavros|diff_planner|traj_server|se3_controller|pointcloud|traj_msg|goal_bridge|rviz' || true"
    info "MAVROS state:"
    ros_exec "$DEV_CONTAINER" \
      "timeout 4 rostopic echo -n 1 /mavros/state 2>/dev/null || true"
  else
    info "ROS master is not reachable."
  fi
}

attach_stack() {
  ensure_prereqs
  tmux_has_session || die "Simulation session '$SESSION_NAME' is not running."
  exec tmux attach-session -t "$SESSION_NAME"
}

arm_vehicle() {
  ensure_prereqs
  need_cmd python3
  require_running_dev_container
  master_is_running || die "ROS master is not running. Start the simulation first."
  [ -f "$ARM_EXECUTOR_HOST" ] ||
    die "Shared arm executor not found: $ARM_EXECUTOR_HOST"

  info "Starting the shared low-latency arm/takeoff/OFFBOARD state machine..."
  ros_exec "$DEV_CONTAINER" \
    "python3 -u - \
      --takeoff-height '$TAKEOFF_HEIGHT' \
      --px4-hover-thrust '$PX4_HOVER_THRUST' \
      --disarmed-prearm-mode '$DISARMED_PREARM_MODE' \
      --takeoff-altitude-field '$TAKEOFF_ALTITUDE_FIELD' \
      --preflight-timeout '$PREFLIGHT_TIMEOUT' \
      --command-timeout '$COMMAND_TIMEOUT' \
      --takeoff-timeout '$TAKEOFF_TIMEOUT' \
      --takeoff-tolerance '$TAKEOFF_TOLERANCE' \
      --takeoff-stable-time '$TAKEOFF_STABLE_TIME' \
      --takeoff-max-vertical-speed '$TAKEOFF_MAX_VERTICAL_SPEED' \
      --odometry-topic /localization/odom \
      --controller-node /se3_controller_node \
      --attitude-setpoint-topic /mavros/setpoint_raw/attitude" \
    < "$ARM_EXECUTOR_HOST"
}

publish_goal() {
  if [ "$#" -lt 3 ] || [ "$#" -gt 4 ]; then
    die "Usage: $0 goal X Y Z [YAW_DEG]"
  fi
  need_cmd python3
  ensure_prereqs
  require_running_dev_container
  [ -f "$GOAL_EXECUTOR_HOST" ] ||
    die "Shared goal executor not found: $GOAL_EXECUTOR_HOST"

  local x="$1" y="$2" z="$3" yaw_spec="${4:-}"
  python3 -c \
    'import math,sys; values=[float(value) for value in sys.argv[1:]]; raise SystemExit(0 if all(math.isfinite(value) for value in values) else 1)' \
    "$@" || {
    die "Goal X, Y, Z, and optional YAW_DEG must all be finite numbers."
  }

  local yaw_arg="" armed_arg=""
  if [ -n "$yaw_spec" ]; then
    yaw_arg="--yaw-deg '$yaw_spec'"
  fi
  if [ "$REQUIRE_ARMED_GOAL" != "true" ]; then
    armed_arg="--allow-disarmed"
  fi

  info "Starting the shared low-latency goal validation and publisher..."
  ros_exec "$DEV_CONTAINER" \
    "python3 -u - '$x' '$y' '$z' \
      $yaw_arg $armed_arg \
      --drone-id '$DRONE_ID' \
      --preflight-timeout '$PREFLIGHT_TIMEOUT' \
      --odometry-topic /localization/odom \
      --controller-node /se3_controller_node \
      --attitude-setpoint-topic /mavros/setpoint_raw/attitude" \
    < "$GOAL_EXECUTOR_HOST"
}

run_waypoint_mission() {
  [ "$#" -eq 1 ] || die "Usage: $0 mission FILE"
  local mission_file="$1"
  need_cmd python3
  ensure_prereqs
  require_running_dev_container
  master_is_running || die "ROS master is not running. Start the simulation first."
  [ -f "$mission_file" ] || die "Mission file not found: $mission_file"
  [ -f "$MISSION_RUNNER_HOST" ] || die "Mission runner not found: $MISSION_RUNNER_HOST"
  [ -f "$MISSION_EXECUTOR_HOST" ] || die "Shared mission executor not found: $MISSION_EXECUTOR_HOST"

  # Both launchers copy and execute these exact same two Python files. All
  # flight decisions live there; this wrapper only supplies the container.
  local container_mission_dir="/tmp/sim2real_mission_$$"
  local container_mission="$container_mission_dir/mission_runtime.json"
  local container_runner="$container_mission_dir/waypoint_mission.py"
  local container_executor="$container_mission_dir/mission_executor.py"
  docker exec -i "$DEV_CONTAINER" mkdir -p "$container_mission_dir" >/dev/null ||
    die "Failed to create the shared mission runtime directory."
  docker cp -- "$mission_file" "$DEV_CONTAINER:$container_mission" >/dev/null ||
    die "Failed to copy the mission file into the simulation container."
  docker cp -- "$MISSION_RUNNER_HOST" "$DEV_CONTAINER:$container_runner" >/dev/null ||
    die "Failed to copy the shared waypoint runner into the simulation container."
  docker cp -- "$MISSION_EXECUTOR_HOST" "$DEV_CONTAINER:$container_executor" >/dev/null ||
    die "Failed to copy the shared mission executor into the simulation container."

  local mission_rc=0
  if ros_exec "$DEV_CONTAINER" \
    "python3 -u '$container_executor' '$container_mission' \
      --drone-id '$DRONE_ID' \
      --default-takeoff-height '$TAKEOFF_HEIGHT' \
      --preflight-timeout '$PREFLIGHT_TIMEOUT' \
      --command-timeout '$COMMAND_TIMEOUT' \
      --takeoff-timeout '$TAKEOFF_TIMEOUT' \
      --takeoff-tolerance '$TAKEOFF_TOLERANCE' \
      --takeoff-stable-time '$TAKEOFF_STABLE_TIME' \
      --takeoff-max-vertical-speed '$TAKEOFF_MAX_VERTICAL_SPEED'"; then
    mission_rc=0
  else
    mission_rc=$?
  fi
  docker exec -i "$DEV_CONTAINER" \
    rm -f "$container_mission" "$container_runner" "$container_executor" \
    >/dev/null 2>&1 || true
  docker exec -i "$DEV_CONTAINER" rmdir "$container_mission_dir" \
    >/dev/null 2>&1 || true

  if [ "$mission_rc" -eq 10 ]; then
    warn "Shared mission stopped because manual takeover was detected."
    return 10
  fi
  if [ "$mission_rc" -ne 0 ]; then
    warn "Shared mission failed; no wrapper-specific recovery action was added."
    return "$mission_rc"
  fi
  info "Shared mission completed successfully."
}

shell_dev() {
  exec "$DEV_CONTAINER_SCRIPT" shell
}

main() {
  local action="${1:-}"
  shift || true

  validate_bool_config

  case "$action" in
    build)
      build_overlay
      ;;
    test)
      test_overlay
      ;;
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
    shell)
      shell_dev
      ;;
    *)
      usage
      exit 1
      ;;
  esac
}

main "$@"
