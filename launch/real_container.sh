#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE_NAME="${IMAGE_NAME:-uav_autonomy_real:latest}"
CONTAINER_NAME="${CONTAINER_NAME:-uav_autonomy_real}"
REAL_SESSION_NAME="${REAL_SESSION_NAME:-real_px4_stack}"
FCU_DEVICE="${FCU_DEVICE:-/dev/ttyACM0}"
DISPLAY_VALUE="${DISPLAY:-:0}"
REQUESTED_DRONE_ID="${DRONE_ID:-0}"
FCU_URL="${FCU_URL:-}"
GCS_URL="${GCS_URL:-}"
RUNTIME_DIR="${RUNTIME_DIR:-$PROJECT_ROOT/runtime}"
EXTRA_DOCKER_ARGS="${EXTRA_DOCKER_ARGS:-}"
PLANNER_PLUGIN_PATH="${SIM2REAL_PLANNER_PLUGIN_PATH:-}"

usage() {
  cat <<'EOF'
Usage: real_container.sh {build|run|stop|rm|restart|shell|status} [--force]

This launcher owns the Jetson onboard real-flight image and container. Run it
on the Jetson only. The ground station uses ground_station_container.sh.

run is idempotent when the existing container layout is current. Commands that
would stop, replace or remove a container refuse while a real-flight stack or
recorder is active. --force is reserved for emergency recovery or maintenance
after the aircraft has been made safe by the pilot.
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

need_cmd() {
  command -v "$1" >/dev/null 2>&1 || die "Missing command: $1"
}

ensure_prereqs() {
  need_cmd docker
  need_cmd realpath
}

validate_single_vehicle_mode() {
  [ "$REQUESTED_DRONE_ID" = "0" ] ||
    die "Only one vehicle is supported; DRONE_ID must be unset or 0 (got: $REQUESTED_DRONE_ID)."
}

append_extra_args() {
  local value="$1" destination_name="$2"
  local -n output_array_ref="$destination_name"
  [ -n "$value" ] || return 0
  [[ "$value" != *$'\n'* && "$value" != *$'\r'* ]] ||
    die "EXTRA_DOCKER_ARGS must be a single line of whitespace-separated Docker arguments."
  local -a parsed=()
  read -r -a parsed <<<"$value"
  [ "${#parsed[@]}" -gt 0 ] ||
    die "EXTRA_DOCKER_ARGS did not contain a usable Docker argument."
  output_array_ref+=("${parsed[@]}")
}

append_planner_plugin_mounts() {
  local destination_name="$1"
  # shellcheck disable=SC2178
  local -n output_array_ref="$destination_name"
  [ -n "$PLANNER_PLUGIN_PATH" ] || return 0
  local entry mount_source mount_target
  local -a entries=()
  IFS=: read -r -a entries <<<"$PLANNER_PLUGIN_PATH"
  for entry in "${entries[@]}"; do
    [ -n "$entry" ] || continue
    [[ "$entry" = /* ]] ||
      die "SIM2REAL_PLANNER_PLUGIN_PATH entries must be absolute: $entry"
    [ -e "$entry" ] ||
      die "Planner plugin search path does not exist: $entry"
    if [ -f "$entry" ]; then
      mount_source="$(realpath -e "$(dirname "$entry")")"
      mount_target="$(dirname "$entry")"
    else
      mount_source="$(realpath -e "$entry")"
      mount_target="$entry"
    fi
    output_array_ref+=(-v "$mount_source:$mount_target:ro")
  done
}

container_exists() {
  docker container inspect "$CONTAINER_NAME" >/dev/null 2>&1
}

container_running() {
  [ "$(docker container inspect -f '{{.State.Running}}' "$CONTAINER_NAME" 2>/dev/null || echo false)" = "true" ]
}

container_processes_active() {
  container_running || return 1
  docker top "$CONTAINER_NAME" -eo args 2>/dev/null |
    grep -Eq '(^|[ /])(roscore|rosmaster|mavros_node|fastlio_mapping|livox_ros_driver2_node|se3_controller_node|diff_planner_node|fast_planner_node|super_backend_adapter_node|traj_server)([[:space:]]|$)|localization_guard\.py|planner_backend_runner\.py|planner_(manager|gateway|visualization)\.py|command_gateway\.py|(diff|fast)_backend_adapter|sim2real_(diff|fast)_adapter|sim2real_super_adapter|super_planner/fsm_node|/rosbag[[:space:]]+record|/rosbag/record[[:space:]]'
}

real_stack_active() {
  if command -v tmux >/dev/null 2>&1 &&
    tmux has-session -t "$REAL_SESSION_NAME" >/dev/null 2>&1; then
    return 0
  fi
  container_processes_active
}

mavros_reports_armed() {
  container_running || return 1
  docker exec -i "$CONTAINER_NAME" bash -lc '
    export ROS_MASTER_URI=http://127.0.0.1:11311
    unset ROS_HOSTNAME
    source /opt/ros/noetic/setup.bash
    [ ! -f /opt/uav-autonomy-aio/planning/workspaces/control_ws/devel/setup.bash ] ||
      source /opt/uav-autonomy-aio/planning/workspaces/control_ws/devel/setup.bash
    timeout 4 rostopic echo -n 1 /mavros/state 2>/dev/null | grep -q "armed: True"
  '
}

require_inactive_real_stack() {
  local force="${1:-false}"
  if [ "$force" = "true" ]; then
    warn "--force bypasses the real-flight container interlock. Confirm that the aircraft is safe."
    return 0
  fi
  real_stack_active || return 0
  if mavros_reports_armed; then
    die "Refusing to mutate '$CONTAINER_NAME': PX4 reports armed. Land/disarm or use RC first."
  fi
  die "Refusing to mutate '$CONTAINER_NAME' while the real stack or recorder is active. Run './launch/real.sh stop' first."
}

validate_container_inputs() {
  local runtime_real project_real
  [ -f "$PROJECT_ROOT/deployment/config/livox/MID360s_config.json" ] ||
    die "Livox configuration is missing."
  [ -f "$PROJECT_ROOT/deployment/config/controller.yaml" ] ||
    die "Controller configuration is missing."
  runtime_real="$(realpath -m "$RUNTIME_DIR")"
  project_real="$(realpath -e "$PROJECT_ROOT")"
  if [ "$runtime_real" = "/" ] || [ "$runtime_real" = "$project_real" ]; then
    die "RUNTIME_DIR must be a dedicated directory, not '$runtime_real'."
  fi
  RUNTIME_DIR="$runtime_real"
  mkdir -p "$RUNTIME_DIR/flight_bags" "$RUNTIME_DIR/tmp"
}

ensure_image() {
  local planner_layout=""
  planner_layout="$(
    docker image inspect \
      -f '{{index .Config.Labels "io.sim2real.planner-workspaces"}}' \
      "$IMAGE_NAME" 2>/dev/null || true
  )"
  if [ "$planner_layout" != "v2" ]; then
    build_image
  fi
}

mount_source_for() {
  local mount_destination="$1"
  docker container inspect --format "{{range .Mounts}}{{if eq .Destination \"$mount_destination\"}}{{.Source}}{{end}}{{end}}" "$CONTAINER_NAME"
}

mount_rw_for() {
  local mount_destination="$1"
  docker container inspect --format "{{range .Mounts}}{{if eq .Destination \"$mount_destination\"}}{{.RW}}{{end}}{{end}}" "$CONTAINER_NAME"
}

container_layout_current() {
  local expected_image actual_image expected_bags expected_tmp expected_livox expected_controller
  expected_image="$(docker image inspect -f '{{.Id}}' "$IMAGE_NAME" 2>/dev/null)" || return 1
  actual_image="$(docker container inspect -f '{{.Image}}' "$CONTAINER_NAME" 2>/dev/null)" || return 1
  [ "$actual_image" = "$expected_image" ] || return 1
  expected_bags="$(realpath -m "$RUNTIME_DIR/flight_bags")"
  expected_tmp="$(realpath -m "$RUNTIME_DIR/tmp")"
  expected_livox="$(realpath -e "$PROJECT_ROOT/deployment/config/livox/MID360s_config.json")"
  expected_controller="$(realpath -e "$PROJECT_ROOT/deployment/config/controller.yaml")"
  [ "$(realpath -m "$(mount_source_for /root/flight_bags)")" = "$expected_bags" ] || return 1
  [ "$(realpath -m "$(mount_source_for /root/tmp)")" = "$expected_tmp" ] || return 1
  [ "$(realpath -m "$(mount_source_for /root/livox_ws/src/livox_ros_driver2/config/MID360s_config.json)")" = "$expected_livox" ] || return 1
  [ "$(realpath -m "$(mount_source_for /root/deployment/controller.yaml)")" = "$expected_controller" ] || return 1
  [ "$(mount_rw_for /root/flight_bags)" = "true" ] || return 1
  [ "$(mount_rw_for /root/tmp)" = "true" ] || return 1
  [ "$(mount_rw_for /root/livox_ws/src/livox_ros_driver2/config/MID360s_config.json)" = "false" ] || return 1
  [ "$(mount_rw_for /root/deployment/controller.yaml)" = "false" ] || return 1
  [ "$(docker container inspect -f '{{.HostConfig.Init}}' "$CONTAINER_NAME")" = "true" ] || return 1
  [ "$(docker container inspect -f '{{.HostConfig.NetworkMode}}' "$CONTAINER_NAME")" = "host" ] || return 1
  [ "$(docker container inspect -f '{{.HostConfig.IpcMode}}' "$CONTAINER_NAME")" = "host" ] || return 1
  [ "$(docker container inspect -f '{{.HostConfig.Privileged}}' "$CONTAINER_NAME")" = "true" ] || return 1
  docker container inspect --format '{{range .Config.Env}}{{println .}}{{end}}' "$CONTAINER_NAME" |
    grep -qx 'DRONE_ID=0' || return 1
  docker container inspect --format '{{range .Config.Env}}{{println .}}{{end}}' "$CONTAINER_NAME" |
    grep -Fxqx "SIM2REAL_PLANNER_PLUGIN_PATH=$PLANNER_PLUGIN_PATH" || return 1
}

build_image() {
  ensure_prereqs
  info "Building image: $IMAGE_NAME"
  docker build -t "$IMAGE_NAME" -f "$PROJECT_ROOT/deployment/Dockerfile" "$PROJECT_ROOT"
}

remove_container_unchecked() {
  if container_exists; then
    info "Removing container: $CONTAINER_NAME"
    docker rm -f "$CONTAINER_NAME" >/dev/null ||
      die "Failed to remove container: $CONTAINER_NAME"
  else
    info "Container does not exist: $CONTAINER_NAME"
  fi
}

create_container() {
  validate_container_inputs

  local -a docker_args
  docker_args=(
    -d
    --init
    --name "$CONTAINER_NAME"
    --network host
    --ipc host
    --privileged
    -e "DRONE_ID=0"
    -e "SIM2REAL_PLANNER_PLUGIN_PATH=$PLANNER_PLUGIN_PATH"
    -e "SIM2REAL_RUNTIME_MODE=real"
    -e "DISPLAY=$DISPLAY_VALUE"
    -e "QT_X11_NO_MITSHM=1"
    -v /etc/localtime:/etc/localtime:ro
    -v "$RUNTIME_DIR/flight_bags:/root/flight_bags"
    -v "$RUNTIME_DIR/tmp:/root/tmp"
    -v "$PROJECT_ROOT/deployment/config/livox/MID360s_config.json:/root/livox_ws/src/livox_ros_driver2/config/MID360s_config.json:ro"
    -v "$PROJECT_ROOT/deployment/config/controller.yaml:/root/deployment/controller.yaml:ro"
  )
  append_planner_plugin_mounts docker_args

  [ -z "$FCU_URL" ] || docker_args+=(-e "FCU_URL=$FCU_URL")
  [ -z "$GCS_URL" ] || docker_args+=(-e "GCS_URL=$GCS_URL")

  if [ -e "$FCU_DEVICE" ]; then
    docker_args+=(--device "$FCU_DEVICE:$FCU_DEVICE")
  else
    warn "FCU device not found on host: $FCU_DEVICE"
  fi
  [ ! -d /tmp/.X11-unix ] || docker_args+=(-v /tmp/.X11-unix:/tmp/.X11-unix)
  [ ! -f "$HOME/.Xauthority" ] || docker_args+=(-v "$HOME/.Xauthority:/root/.Xauthority:ro")
  append_extra_args "$EXTRA_DOCKER_ARGS" docker_args

  info "Starting container: $CONTAINER_NAME"
  docker run "${docker_args[@]}" "$IMAGE_NAME" tail -f /dev/null >/dev/null
}

run_container() {
  local force="${1:-false}"
  ensure_prereqs
  validate_single_vehicle_mode
  validate_container_inputs
  ensure_image

  if container_exists; then
    if container_layout_current; then
      if container_running; then
        info "Container is already running: $CONTAINER_NAME"
      else
        require_inactive_real_stack "$force"
        info "Starting existing container: $CONTAINER_NAME"
        docker start "$CONTAINER_NAME" >/dev/null
      fi
      return
    fi
    require_inactive_real_stack "$force"
    warn "Container image or mounts are stale; recreating it."
    remove_container_unchecked
  else
    require_inactive_real_stack "$force"
  fi
  create_container
}

stop_container() {
  local force="${1:-false}"
  ensure_prereqs
  if container_running; then
    require_inactive_real_stack "$force"
    info "Stopping container: $CONTAINER_NAME"
    docker stop "$CONTAINER_NAME" >/dev/null
  else
    info "Container is not running: $CONTAINER_NAME"
  fi
}

remove_container() {
  local force="${1:-false}"
  ensure_prereqs
  if container_exists; then
    require_inactive_real_stack "$force"
  fi
  remove_container_unchecked
}

restart_container() {
  local force="${1:-false}"
  ensure_prereqs
  validate_single_vehicle_mode
  validate_container_inputs
  ensure_image
  require_inactive_real_stack "$force"
  remove_container_unchecked
  create_container
}

shell_container() {
  run_container false
  exec docker exec -it "$CONTAINER_NAME" bash
}

status_container() {
  ensure_prereqs
  docker ps -a --filter "name=^/${CONTAINER_NAME}$" --format 'table {{.Names}}\t{{.Image}}\t{{.Status}}'
}

parse_force_arg() {
  if [ "$#" -eq 0 ]; then
    printf 'false\n'
  elif [ "$#" -eq 1 ] && [ "$1" = "--force" ]; then
    printf 'true\n'
  else
    usage >&2
    return 1
  fi
}

main() {
  local action="${1:-}"
  shift || true
  local force

  case "$action" in
    build)
      [ "$#" -eq 0 ] || die "Usage: $0 build"
      build_image
      ;;
    run)
      force="$(parse_force_arg "$@")"
      run_container "$force"
      ;;
    stop)
      force="$(parse_force_arg "$@")"
      stop_container "$force"
      ;;
    rm)
      force="$(parse_force_arg "$@")"
      remove_container "$force"
      ;;
    restart)
      force="$(parse_force_arg "$@")"
      restart_container "$force"
      ;;
    shell)
      [ "$#" -eq 0 ] || die "Usage: $0 shell"
      shell_container
      ;;
    status)
      [ "$#" -eq 0 ] || die "Usage: $0 status"
      status_container
      ;;
    *)
      usage
      exit 1
      ;;
  esac
}

main "$@"
