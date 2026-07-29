#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

DOCKERFILE="$PROJECT_ROOT/simulation/Dockerfile"
VERSIONS_FILE="$PROJECT_ROOT/simulation/versions.env"

# shellcheck disable=SC1090
source "$VERSIONS_FILE"

IMAGE_NAME="${SIM_DEV_IMAGE:-diff_planner_px4_sim:noetic}"
CONTAINER_NAME="${SIM_DEV_CONTAINER:-diff_planner_px4_sim}"
SOURCE_HOST="${SIM_SOURCE_HOST:-$PROJECT_ROOT/third_party/Diff-Planner-PX4}"
COMMON_SOURCE_HOST="${SIM_COMMON_SOURCE_HOST:-$PROJECT_ROOT/common}"
ADAPTER_SOURCE_HOST="${SIM_ADAPTER_SOURCE_HOST:-$PROJECT_ROOT/simulation/ros_pkgs/sim2real_simulation}"
PROJECT_SOURCE_HOST="${SIM_PROJECT_SOURCE_HOST:-$PROJECT_ROOT}"
SIMULATION_CONFIG_HOST="${SIM_CONFIG_HOST:-$PROJECT_ROOT/simulation/config}"
RUNTIME_HOST="${SIM_RUNTIME_HOST:-$PROJECT_ROOT/runtime/simulation}"
WORKSPACE_HOST="${SIM_WORKSPACE_HOST:-$RUNTIME_HOST/catkin_ws}"
PLANNER_WORKSPACES_HOST="${SIM_PLANNER_WORKSPACES_HOST:-$PROJECT_ROOT/planning/workspaces}"
PLANNER_PLUGIN_PATH="${SIM2REAL_PLANNER_PLUGIN_PATH:-}"
WORKSPACE_CONTAINER="${SIM_WORKSPACE_CONTAINER:-/workspaces/sim2real_ws}"
SOURCE_CONTAINER="$WORKSPACE_CONTAINER/src/Diff-Planner-PX4"
COMMON_SOURCE_CONTAINER="$WORKSPACE_CONTAINER/src/sim2real_common"
ADAPTER_SOURCE_CONTAINER="$WORKSPACE_CONTAINER/src/sim2real_simulation"
PROJECT_SOURCE_CONTAINER="${SIM_PROJECT_SOURCE_CONTAINER:-/opt/uav-autonomy-aio}"
PLANNER_WORKSPACES_CONTAINER="$PROJECT_SOURCE_CONTAINER/planning/workspaces"
SIMULATION_CONFIG_CONTAINER="${SIM_CONFIG_CONTAINER:-/etc/sim2real/simulation}"
RUNTIME_CONTAINER="${SIM_RUNTIME_CONTAINER:-/root/simulation_runtime}"
DISPLAY_VALUE="${DISPLAY:-:0}"
EXTRA_DOCKER_ARGS="${SIM_EXTRA_DOCKER_ARGS:-}"
EXTRA_BUILD_ARGS="${SIM_DOCKER_BUILD_ARGS:-}"
IMAGE_BUILD_JOBS="${SIM_IMAGE_BUILD_JOBS:-4}"
GPU_MODE_REQUESTED="${SIM_GPU_MODE:-auto}"

usage() {
  cat <<'EOF'
Usage: sim_container.sh {build|run|stop|restart|recreate|rm|shell|status|verify} [--force]

Builds and manages the repository-owned PX4/Gazebo/Mid360 simulation image.
The repository is mounted read-only and the four generated planner workspaces
are mounted read-write below planning/workspaces. Legacy source mounts are
retained for compatibility but are not used to overlay Fast and Diff together.
No pre-existing ros_noetic container is used. SIM_GPU_MODE defaults to auto
and accepts auto|nvidia|dri|none.
Container mutation is refused while a simulation stack is active. --force is
reserved for recovery after normal './launch/sim.sh stop' cannot be used.
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

resolve_gpu_mode() {
  local nvidia_probe=""
  case "$GPU_MODE_REQUESTED" in
    auto)
      if command -v nvidia-smi >/dev/null 2>&1; then
        if nvidia_probe="$(nvidia-smi -L 2>&1)"; then
          if docker info --format '{{json .Runtimes}}' 2>/dev/null | grep -q '"nvidia"'; then
            echo nvidia
            return
          fi
        elif grep -qi 'driver/library version mismatch' <<<"$nvidia_probe"; then
          die "Host NVIDIA driver/library versions do not match. Reboot the host before starting Gazebo; refusing to silently fall back to software rendering."
        fi
      fi
      if [ -d /dev/dri ]; then
        echo dri
      else
        echo none
      fi
      ;;
    nvidia)
      command -v nvidia-smi >/dev/null 2>&1 || die "SIM_GPU_MODE=nvidia requires nvidia-smi on the host."
      nvidia-smi -L >/dev/null 2>&1 || die "SIM_GPU_MODE=nvidia cannot access a host NVIDIA GPU."
      docker info --format '{{json .Runtimes}}' 2>/dev/null | grep -q '"nvidia"' ||
        die "SIM_GPU_MODE=nvidia requires NVIDIA Container Toolkit."
      echo nvidia
      ;;
    dri)
      [ -d /dev/dri ] || die "SIM_GPU_MODE=dri requires /dev/dri on the host."
      echo dri
      ;;
    none)
      echo none
      ;;
    *)
      die "SIM_GPU_MODE must be one of auto, nvidia, dri, or none."
      ;;
  esac
}

container_exists() {
  docker inspect "$CONTAINER_NAME" >/dev/null 2>&1
}

container_running() {
  [ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER_NAME" 2>/dev/null || echo false)" = "true" ]
}

active_simulation_detected() {
  local marker
  for marker in "$RUNTIME_HOST"/active/*.owner; do
    [ ! -e "$marker" ] || return 0
  done

  container_running || return 1
  docker top "$CONTAINER_NAME" -eo args 2>/dev/null |
    grep -Eq '(^|[ /])(roscore|rosmaster|gzserver|gzclient|mavros_node|px4|se3_controller_node|diff_planner_node|fast_planner_node|traj_server)([[:space:]]|$)|planner_backend_runner\.py|planner_(manager|gateway|visualization)\.py|command_gateway\.py|(diff|fast)_backend_adapter|sim2real_(diff|fast)_adapter|outdoor_mid360\.launch|/rosbag[[:space:]]+record|/rosbag/record[[:space:]]'
}

require_inactive_simulation() {
  local force="${1:-false}"
  if [ "$force" = "true" ]; then
    warn "--force bypasses the active-simulation container interlock."
    return 0
  fi
  active_simulation_detected &&
    die "Refusing to mutate '$CONTAINER_NAME' while a simulation stack is active. Run './launch/sim.sh stop' first."
  return 0
}

append_extra_args() {
  local value="$1" destination_name="$2" variable_name="$3"
  local -n output_array_ref="$destination_name"
  [ -n "$value" ] || return 0
  [[ "$value" != *$'\n'* && "$value" != *$'\r'* ]] ||
    die "$variable_name must be a single line of whitespace-separated Docker arguments."
  local -a parsed=()
  read -r -a parsed <<<"$value"
  [ "${#parsed[@]}" -gt 0 ] ||
    die "$variable_name did not contain a usable Docker argument."
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

canonical_path() {
  realpath -m "$1"
}

mount_source_for() {
  local mount_destination="$1"
  docker inspect --format "{{range .Mounts}}{{if eq .Destination \"$mount_destination\"}}{{.Source}}{{end}}{{end}}" "$CONTAINER_NAME"
}

mount_rw_for() {
  local mount_destination="$1"
  docker inspect --format "{{range .Mounts}}{{if eq .Destination \"$mount_destination\"}}{{.RW}}{{end}}{{end}}" "$CONTAINER_NAME"
}

verify_live_bind_mount() {
  local label="$1"
  local host_path="$2"
  local container_path="$3"
  local host_identity=""
  local container_identity=""

  [ -e "$host_path" ] ||
    die "Host $label mount source is missing: $host_path. Run '$0 recreate'."

  # Docker inspect only preserves the configured source path. If that host
  # directory is deleted and recreated while the container is running, the
  # bind mount still refers to the deleted inode even though inspect prints
  # the same path. Compare the live filesystem objects to catch that state.
  container_running || return 0
  host_identity="$(stat -Lc '%d:%i' -- "$host_path")" ||
    die "Cannot inspect host $label mount source: $host_path"
  container_identity="$(
    docker exec "$CONTAINER_NAME" \
      stat -Lc '%d:%i' -- "$container_path" 2>/dev/null
  )" ||
    die "Container $label mount target is inaccessible: $container_path. Run '$0 recreate'."
  [ "$container_identity" = "$host_identity" ] ||
    die "Container $label mount is detached from '$host_path'. Run '$0 recreate'."
}

verify_mounts() {
  container_exists || die "Container '$CONTAINER_NAME' does not exist. Run '$0 run' first."

  local expected_source expected_common expected_adapter expected_project expected_planner_workspaces
  local expected_config expected_workspace expected_runtime
  local actual_source actual_common actual_adapter actual_project actual_planner_workspaces
  local actual_config actual_workspace actual_runtime
  local actual_source_rw actual_common_rw actual_adapter_rw actual_project_rw actual_planner_workspaces_rw
  local actual_config_rw actual_workspace_rw actual_runtime_rw
  local expected_image_id actual_image_id
  local actual_workspace_env actual_plugin_path expected_gpu_mode actual_gpu_mode actual_gpu_request
  expected_source="$(canonical_path "$SOURCE_HOST")"
  expected_common="$(canonical_path "$COMMON_SOURCE_HOST")"
  expected_adapter="$(canonical_path "$ADAPTER_SOURCE_HOST")"
  expected_project="$(canonical_path "$PROJECT_SOURCE_HOST")"
  expected_planner_workspaces="$(canonical_path "$PLANNER_WORKSPACES_HOST")"
  expected_config="$(canonical_path "$SIMULATION_CONFIG_HOST")"
  expected_workspace="$(canonical_path "$WORKSPACE_HOST")"
  expected_runtime="$(canonical_path "$RUNTIME_HOST")"
  actual_source="$(mount_source_for "$SOURCE_CONTAINER")"
  actual_common="$(mount_source_for "$COMMON_SOURCE_CONTAINER")"
  actual_adapter="$(mount_source_for "$ADAPTER_SOURCE_CONTAINER")"
  actual_project="$(mount_source_for "$PROJECT_SOURCE_CONTAINER")"
  actual_planner_workspaces="$(mount_source_for "$PLANNER_WORKSPACES_CONTAINER")"
  actual_config="$(mount_source_for "$SIMULATION_CONFIG_CONTAINER")"
  actual_workspace="$(mount_source_for "$WORKSPACE_CONTAINER")"
  actual_runtime="$(mount_source_for "$RUNTIME_CONTAINER")"
  actual_source_rw="$(mount_rw_for "$SOURCE_CONTAINER")"
  actual_common_rw="$(mount_rw_for "$COMMON_SOURCE_CONTAINER")"
  actual_adapter_rw="$(mount_rw_for "$ADAPTER_SOURCE_CONTAINER")"
  actual_project_rw="$(mount_rw_for "$PROJECT_SOURCE_CONTAINER")"
  actual_planner_workspaces_rw="$(mount_rw_for "$PLANNER_WORKSPACES_CONTAINER")"
  actual_config_rw="$(mount_rw_for "$SIMULATION_CONFIG_CONTAINER")"
  actual_workspace_rw="$(mount_rw_for "$WORKSPACE_CONTAINER")"
  actual_runtime_rw="$(mount_rw_for "$RUNTIME_CONTAINER")"
  actual_workspace_env="$(docker inspect --format '{{range .Config.Env}}{{println .}}{{end}}' "$CONTAINER_NAME" | sed -n 's/^SIM_WORKSPACE_CONTAINER=//p' | tail -n 1)"
  actual_plugin_path="$(docker inspect --format '{{range .Config.Env}}{{println .}}{{end}}' "$CONTAINER_NAME" | sed -n 's/^SIM2REAL_PLANNER_PLUGIN_PATH=//p' | tail -n 1)"
  expected_gpu_mode="$(resolve_gpu_mode)" || return 1
  actual_gpu_mode="$(docker inspect --format '{{range .Config.Env}}{{println .}}{{end}}' "$CONTAINER_NAME" | sed -n 's/^SIM_GPU_MODE_RESOLVED=//p' | tail -n 1)"

  [ "$(canonical_path "$actual_source")" = "$expected_source" ] || {
    die "Container source mount is stale: '$actual_source'. Run '$0 recreate'."
  }
  [ "$(canonical_path "$actual_common")" = "$expected_common" ] || {
    die "Container common source mount is stale: '$actual_common'. Run '$0 recreate'."
  }
  [ "$(canonical_path "$actual_adapter")" = "$expected_adapter" ] || {
    die "Container simulation adapter mount is stale: '$actual_adapter'. Run '$0 recreate'."
  }
  [ "$(canonical_path "$actual_project")" = "$expected_project" ] || {
    die "Container repository mount is stale: '$actual_project'. Run '$0 recreate'."
  }
  [ "$(canonical_path "$actual_planner_workspaces")" = "$expected_planner_workspaces" ] || {
    die "Container planner-workspaces mount is stale: '$actual_planner_workspaces'. Run '$0 recreate'."
  }
  [ "$(canonical_path "$actual_config")" = "$expected_config" ] || {
    die "Container simulation config mount is stale: '$actual_config'. Run '$0 recreate'."
  }
  [ "$(canonical_path "$actual_workspace")" = "$expected_workspace" ] || {
    die "Container workspace mount is stale: '$actual_workspace'. Run '$0 recreate'."
  }
  [ "$(canonical_path "$actual_runtime")" = "$expected_runtime" ] || {
    die "Container runtime mount is stale: '$actual_runtime'. Run '$0 recreate'."
  }
  [ "$actual_source_rw" = "false" ] || {
    die "Container source mount must be read-only. Run '$0 recreate'."
  }
  [ "$actual_common_rw" = "false" ] || {
    die "Container common source mount must be read-only. Run '$0 recreate'."
  }
  [ "$actual_adapter_rw" = "false" ] || {
    die "Container simulation adapter mount must be read-only. Run '$0 recreate'."
  }
  [ "$actual_project_rw" = "false" ] || {
    die "Container repository mount must be read-only. Run '$0 recreate'."
  }
  [ "$actual_planner_workspaces_rw" = "true" ] || {
    die "Container planner workspaces must be writable. Run '$0 recreate'."
  }
  [ "$actual_config_rw" = "false" ] || {
    die "Container simulation config mount must be read-only. Run '$0 recreate'."
  }
  [ "$actual_workspace_rw" = "true" ] || {
    die "Container workspace mount must be writable. Run '$0 recreate'."
  }
  [ "$actual_runtime_rw" = "true" ] || {
    die "Container runtime mount must be writable. Run '$0 recreate'."
  }

  verify_live_bind_mount "runtime" "$RUNTIME_HOST" "$RUNTIME_CONTAINER"
  verify_live_bind_mount "overlay workspace" "$WORKSPACE_HOST" "$WORKSPACE_CONTAINER"
  verify_live_bind_mount "repository" "$PROJECT_SOURCE_HOST" "$PROJECT_SOURCE_CONTAINER"
  verify_live_bind_mount "planner workspaces" "$PLANNER_WORKSPACES_HOST" "$PLANNER_WORKSPACES_CONTAINER"
  verify_live_bind_mount "Diff source" "$SOURCE_HOST" "$SOURCE_CONTAINER"
  verify_live_bind_mount "common source" "$COMMON_SOURCE_HOST" "$COMMON_SOURCE_CONTAINER"
  verify_live_bind_mount "simulation adapter" "$ADAPTER_SOURCE_HOST" "$ADAPTER_SOURCE_CONTAINER"
  verify_live_bind_mount "simulation config" "$SIMULATION_CONFIG_HOST" "$SIMULATION_CONFIG_CONTAINER"

  [ "$actual_workspace_env" = "$WORKSPACE_CONTAINER" ] || {
    die "Container SIM_WORKSPACE_CONTAINER is stale: '$actual_workspace_env'. Run '$0 recreate'."
  }
  [ "$actual_plugin_path" = "$PLANNER_PLUGIN_PATH" ] || {
    die "Container planner plugin path is stale. Run '$0 recreate'."
  }
  [ "$actual_gpu_mode" = "$expected_gpu_mode" ] || {
    die "Container GPU mode is stale: '${actual_gpu_mode:-unset}' (expected '$expected_gpu_mode'). Run '$0 recreate'."
  }
  if [ "$expected_gpu_mode" = "nvidia" ]; then
    actual_gpu_request="$(docker inspect --format '{{json .HostConfig.DeviceRequests}}' "$CONTAINER_NAME")"
    grep -q '"gpu"' <<<"$actual_gpu_request" || {
      die "Container is missing its NVIDIA device request. Run '$0 recreate'."
    }
  fi

  expected_image_id="$(docker image inspect -f '{{.Id}}' "$IMAGE_NAME")"
  actual_image_id="$(docker inspect -f '{{.Image}}' "$CONTAINER_NAME")"
  [ "$actual_image_id" = "$expected_image_id" ] || {
    die "Container image is stale. Run '$0 recreate' after rebuilding '$IMAGE_NAME'."
  }

  info "Source mount: $expected_source -> $SOURCE_CONTAINER (read-only)"
  info "Common mount: $expected_common -> $COMMON_SOURCE_CONTAINER (read-only)"
  info "Simulation adapter: $expected_adapter -> $ADAPTER_SOURCE_CONTAINER (read-only)"
  info "Repository: $expected_project -> $PROJECT_SOURCE_CONTAINER (read-only)"
  info "Planner workspaces: $expected_planner_workspaces -> $PLANNER_WORKSPACES_CONTAINER"
  info "Simulation config: $expected_config -> $SIMULATION_CONFIG_CONTAINER (read-only)"
  info "Overlay workspace: $expected_workspace -> $WORKSPACE_CONTAINER"
  info "Runtime data: $expected_runtime -> $RUNTIME_CONTAINER"
  info "Graphics acceleration: $expected_gpu_mode"
}

build_image() {
  need_cmd docker
  [[ "$IMAGE_BUILD_JOBS" =~ ^[1-9][0-9]*$ ]] || {
    die "SIM_IMAGE_BUILD_JOBS must be a positive integer."
  }

  local -a build_args
  build_args=(
    --build-arg "PX4_REPOSITORY=$PX4_REPOSITORY"
    --build-arg "PX4_TAG=$PX4_TAG"
    --build-arg "PX4_COMMIT=$PX4_COMMIT"
    --build-arg "MID360_REPOSITORY=$MID360_REPOSITORY"
    --build-arg "MID360_COMMIT=$MID360_COMMIT"
    --build-arg "BUILD_JOBS=$IMAGE_BUILD_JOBS"
  )

  append_extra_args "$EXTRA_BUILD_ARGS" build_args SIM_DOCKER_BUILD_ARGS

  info "Building repository-owned simulation image: $IMAGE_NAME"
  docker build "${build_args[@]}" -t "$IMAGE_NAME" -f "$DOCKERFILE" "$PROJECT_ROOT"
}

ensure_image() {
  local planner_layout=""
  planner_layout="$(
    docker image inspect \
      -f '{{index .Config.Labels "io.sim2real.planner-workspaces"}}' \
      "$IMAGE_NAME" 2>/dev/null || true
  )"
  if [ "$planner_layout" != "v1" ]; then
    info "Simulation image is missing or predates planner workspace isolation; building it from $DOCKERFILE"
    build_image
  fi
}

verify_environment() {
  local gpu_mode
  gpu_mode="$(resolve_gpu_mode)"
  docker exec -i \
    -e SIM_WORKSPACE_CONTAINER=/__verify_image_without_host_overlay__ \
    -e "SIM_EXPECTED_GPU_MODE=$gpu_mode" \
    "$CONTAINER_NAME" bash -lc '
    set -e
    source /root/.bashrc
    test "$(realpath "$(rospack find px4)")" = "/opt/PX4-Autopilot"
    test "$(realpath "$(rospack find livox_laser_simulation)")" = "/opt/simulation_ws/src/Mid360_px4_sim_plugin/livox_laser_simulation"
    test -x /opt/PX4-Autopilot/build/px4_sitl_default/bin/px4
    test -e /opt/simulation_ws/devel/lib/liblivox_laser_simulation.so
    test -f /usr/include/nlopt.hpp
    pkg-config --exists nlopt
    case "$SIM_EXPECTED_GPU_MODE" in
      nvidia)
        command -v nvidia-smi >/dev/null
        nvidia-smi -L >/dev/null
        ldconfig -p | grep -q libGLX_nvidia
        ;;
      dri)
        test -d /dev/dri
        ;;
      none) ;;
      *) exit 1 ;;
    esac
  ' || die "The simulation image is incomplete. Rebuild it with '$0 build'."
}

validate_container_inputs() {
  ensure_image
  resolve_gpu_mode >/dev/null
  [ -d "$SOURCE_HOST" ] || die "Diff-Planner source not found: $SOURCE_HOST"
  [ -d "$COMMON_SOURCE_HOST" ] || die "Common ROS package not found: $COMMON_SOURCE_HOST"
  [ -d "$ADAPTER_SOURCE_HOST" ] || die "Simulation adapter package not found: $ADAPTER_SOURCE_HOST"
  [ -x "$PROJECT_SOURCE_HOST/planning/scripts/build_planner_workspaces.sh" ] ||
    die "Planner workspace builder not found below: $PROJECT_SOURCE_HOST"
  [ -d "$SIMULATION_CONFIG_HOST" ] || die "Simulation config not found: $SIMULATION_CONFIG_HOST"
  mkdir -p \
    "$WORKSPACE_HOST/src" \
    "$PLANNER_WORKSPACES_HOST" \
    "$RUNTIME_HOST/runs" \
    "$RUNTIME_HOST/active" \
    "$RUNTIME_HOST/flight_bags" \
    "$RUNTIME_HOST/logs" \
    "$RUNTIME_HOST/ros_logs"
}

create_container() {
  validate_container_inputs

  if command -v xhost >/dev/null 2>&1; then
    xhost +SI:localuser:root >/dev/null 2>&1 || true
  fi

  local gpu_mode
  gpu_mode="$(resolve_gpu_mode)"

  local -a docker_args
  docker_args=(
    -d
    --init
    --name "$CONTAINER_NAME"
    --network host
    --ipc host
    -e "DISPLAY=$DISPLAY_VALUE"
    -e "QT_X11_NO_MITSHM=1"
    -e "PYTHONDONTWRITEBYTECODE=1"
    -e "SIM_WORKSPACE_CONTAINER=$WORKSPACE_CONTAINER"
    -e "SIM2REAL_PROJECT_ROOT=$PROJECT_SOURCE_CONTAINER"
    -e "SIM2REAL_RUNTIME_MODE=simulation"
    -e "SIM2REAL_PLANNER_PLUGIN_PATH=$PLANNER_PLUGIN_PATH"
    -e "SIM_GPU_MODE_RESOLVED=$gpu_mode"
    -v /etc/localtime:/etc/localtime:ro
    -v "$WORKSPACE_HOST:$WORKSPACE_CONTAINER"
    -v "$SOURCE_HOST:$SOURCE_CONTAINER:ro"
    -v "$COMMON_SOURCE_HOST:$COMMON_SOURCE_CONTAINER:ro"
    -v "$ADAPTER_SOURCE_HOST:$ADAPTER_SOURCE_CONTAINER:ro"
    -v "$PROJECT_SOURCE_HOST:$PROJECT_SOURCE_CONTAINER:ro"
    -v "$PLANNER_WORKSPACES_HOST:$PLANNER_WORKSPACES_CONTAINER"
    -v "$SIMULATION_CONFIG_HOST:$SIMULATION_CONFIG_CONTAINER:ro"
    -v "$RUNTIME_HOST:$RUNTIME_CONTAINER"
  )
  append_planner_plugin_mounts docker_args

  case "$gpu_mode" in
    nvidia)
      docker_args+=(
        --gpus all
        -e NVIDIA_VISIBLE_DEVICES=all
        -e NVIDIA_DRIVER_CAPABILITIES=all
        -e __GLX_VENDOR_LIBRARY_NAME=nvidia
      )
      [ ! -d /dev/dri ] || docker_args+=(--device /dev/dri:/dev/dri)
      ;;
    dri)
      docker_args+=(--device /dev/dri:/dev/dri)
      ;;
    none) ;;
  esac

  if [ -d /tmp/.X11-unix ]; then
    docker_args+=(-v /tmp/.X11-unix:/tmp/.X11-unix)
  fi

  if [ -f "$HOME/.Xauthority" ]; then
    docker_args+=(-v "$HOME/.Xauthority:/root/.Xauthority:ro")
  fi

  append_extra_args "$EXTRA_DOCKER_ARGS" docker_args SIM_EXTRA_DOCKER_ARGS

  info "Creating autonomy simulation container: $CONTAINER_NAME (graphics=$gpu_mode)"
  docker run "${docker_args[@]}" "$IMAGE_NAME" tail -f /dev/null >/dev/null
  verify_mounts
  verify_environment
}

run_container() {
  local force="${1:-false}"
  need_cmd docker
  need_cmd realpath
  resolve_gpu_mode >/dev/null

  if container_exists; then
    ensure_image
    if ! (verify_mounts); then
      validate_container_inputs
      require_inactive_simulation "$force"
      warn "Container layout is stale; recreating it for the UAV Autonomy All-in-One workspace."
      remove_container_unchecked
      create_container
      return
    fi
    if container_running; then
      info "Container is already running: $CONTAINER_NAME"
    else
      info "Starting container: $CONTAINER_NAME"
      docker start "$CONTAINER_NAME" >/dev/null
    fi
  else
    create_container
  fi
  verify_environment
}

stop_container() {
  local force="${1:-false}"
  need_cmd docker
  if container_running; then
    require_inactive_simulation "$force"
    info "Stopping container: $CONTAINER_NAME"
    docker stop "$CONTAINER_NAME" >/dev/null
  else
    info "Container is not running: $CONTAINER_NAME"
  fi
}

remove_container_unchecked() {
  need_cmd docker
  if container_exists; then
    info "Removing container: $CONTAINER_NAME"
    docker rm -f "$CONTAINER_NAME" >/dev/null
  else
    info "Container does not exist: $CONTAINER_NAME"
  fi
}

remove_container() {
  local force="${1:-false}"
  if container_exists; then
    require_inactive_simulation "$force"
  fi
  remove_container_unchecked
}

recreate_container() {
  local force="${1:-false}"
  validate_container_inputs
  require_inactive_simulation "$force"
  remove_container_unchecked
  create_container
}

shell_container() {
  run_container false
  exec docker exec -it \
    -e ROS_MASTER_URI=http://127.0.0.1:11311 \
    -e ROS_IP=127.0.0.1 \
    -e PYTHONDONTWRITEBYTECODE=1 \
    "$CONTAINER_NAME" bash -lc "
      unset ROS_HOSTNAME
      source /root/.bashrc
      exec bash
    "
}

status_container() {
  need_cmd docker
  docker ps -a --filter "name=^/${CONTAINER_NAME}$" --format 'table {{.Names}}\t{{.Image}}\t{{.Status}}'
  if container_exists; then
    verify_mounts
  fi
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
    restart)
      force="$(parse_force_arg "$@")"
      require_inactive_simulation "$force"
      stop_container "$force"
      run_container "$force"
      ;;
    recreate)
      force="$(parse_force_arg "$@")"
      recreate_container "$force"
      ;;
    rm)
      force="$(parse_force_arg "$@")"
      remove_container "$force"
      ;;
    shell)
      [ "$#" -eq 0 ] || die "Usage: $0 shell"
      shell_container
      ;;
    status)
      [ "$#" -eq 0 ] || die "Usage: $0 status"
      status_container
      ;;
    verify)
      [ "$#" -eq 0 ] || die "Usage: $0 verify"
      need_cmd docker
      need_cmd realpath
      verify_mounts
      verify_environment
      ;;
    *)
      usage
      exit 1
      ;;
  esac
}

main "$@"
