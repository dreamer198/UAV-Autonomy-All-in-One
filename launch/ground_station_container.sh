#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DOCKERFILE="$PROJECT_ROOT/deployment/ground_station/Dockerfile"
IMAGE_NAME="${GROUND_STATION_IMAGE:-uav_autonomy_ground_station:noetic}"
CONTAINER_NAME="${GROUND_STATION_CONTAINER:-${CONTAINER_NAME:-uav_autonomy_ground_station}}"
DISPLAY_VALUE="${DISPLAY:-:0}"
EXTRA_DOCKER_ARGS="${GROUND_STATION_EXTRA_DOCKER_ARGS:-}"
IMAGE_LAYOUT_VERSION="v1"
IMAGE_SOURCE_LABEL="io.uav-autonomy-aio.ground-station-source-sha256"

usage() {
  cat <<'EOF'
Usage: ground_station_container.sh {build|run|stop|recreate|rm|shell|status|verify}

Builds and manages the lightweight Linux/X11 ground-station container used by
real_rviz.sh.  It never maps an FCU or lidar device and does not contain the
Jetson localization, planning, or control runtime.
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
  need_cmd sha256sum
}

append_extra_args() {
  local value="$1" destination_name="$2"
  local -n output_array_ref="$destination_name"
  [ -n "$value" ] || return 0
  [[ "$value" != *$'\n'* && "$value" != *$'\r'* ]] ||
    die "GROUND_STATION_EXTRA_DOCKER_ARGS must be a single line of whitespace-separated Docker arguments."
  local -a parsed=()
  read -r -a parsed <<<"$value"
  [ "${#parsed[@]}" -gt 0 ] ||
    die "GROUND_STATION_EXTRA_DOCKER_ARGS did not contain a usable Docker argument."
  output_array_ref+=("${parsed[@]}")
}

container_exists() {
  docker container inspect "$CONTAINER_NAME" >/dev/null 2>&1
}

container_running() {
  [ "$(docker container inspect -f '{{.State.Running}}' "$CONTAINER_NAME" 2>/dev/null || echo false)" = "true" ]
}

image_layout_current() {
  local expected_source_hash actual_source_hash
  expected_source_hash="$(compute_image_source_hash)"
  [ "$(
    docker image inspect \
      -f '{{index .Config.Labels "io.uav-autonomy-aio.ground-station-layout"}}' \
      "$IMAGE_NAME" 2>/dev/null || true
  )" = "$IMAGE_LAYOUT_VERSION" ] || return 1
  actual_source_hash="$(
    docker image inspect \
      -f "{{index .Config.Labels \"$IMAGE_SOURCE_LABEL\"}}" \
      "$IMAGE_NAME" 2>/dev/null || true
  )"
  [ "$actual_source_hash" = "$expected_source_hash" ]
}

compute_image_source_hash() {
  local deployment_source="deployment/ground_station"
  local message_source="planning/ros_pkgs/sim2real_planning_msgs"
  [ -d "$PROJECT_ROOT/$deployment_source" ] ||
    die "Ground-station deployment source is missing: $PROJECT_ROOT/$deployment_source"
  [ -d "$PROJECT_ROOT/$message_source" ] ||
    die "Ground-station message source is missing: $PROJECT_ROOT/$message_source"
  (
    cd "$PROJECT_ROOT"
    find "$deployment_source" "$message_source" -type f -print0 |
      sort -z |
      xargs -0 sha256sum |
      sha256sum |
      awk '{print $1}'
  )
}

build_image() {
  local source_hash
  ensure_prereqs
  [ -f "$DOCKERFILE" ] || die "Ground-station Dockerfile not found: $DOCKERFILE"
  source_hash="$(compute_image_source_hash)"
  info "Building ground-station image: $IMAGE_NAME"
  docker build \
    --label "$IMAGE_SOURCE_LABEL=$source_hash" \
    -t "$IMAGE_NAME" \
    -f "$DOCKERFILE" \
    "$PROJECT_ROOT"
}

ensure_image() {
  if ! image_layout_current; then
    info "Ground-station image is missing or stale; building it from $DOCKERFILE"
    build_image
  fi
}

container_layout_current() {
  local expected_image actual_image
  expected_image="$(docker image inspect -f '{{.Id}}' "$IMAGE_NAME" 2>/dev/null)" || return 1
  actual_image="$(docker container inspect -f '{{.Image}}' "$CONTAINER_NAME" 2>/dev/null)" || return 1
  [ "$actual_image" = "$expected_image" ] || return 1
  [ "$(docker container inspect -f '{{.HostConfig.Init}}' "$CONTAINER_NAME")" = "true" ] || return 1
  [ "$(docker container inspect -f '{{.HostConfig.NetworkMode}}' "$CONTAINER_NAME")" = "host" ] || return 1
  [ "$(docker container inspect -f '{{.HostConfig.IpcMode}}' "$CONTAINER_NAME")" = "host" ] || return 1
  [ "$(docker container inspect -f '{{.HostConfig.Privileged}}' "$CONTAINER_NAME")" = "false" ] || return 1
  docker container inspect --format '{{range .Config.Env}}{{println .}}{{end}}' "$CONTAINER_NAME" |
    grep -qx 'SIM2REAL_RUNTIME_MODE=ground_station'
}

verify_environment() {
  container_exists || die "Ground-station container '$CONTAINER_NAME' does not exist. Run '$0 run' first."
  container_running || die "Ground-station container '$CONTAINER_NAME' is not running. Run '$0 run' first."
  container_layout_current ||
    die "Ground-station container '$CONTAINER_NAME' is stale. Run '$0 recreate'."
  docker exec "$CONTAINER_NAME" bash -lc '
    set -e
    source /root/.bashrc
    command -v rviz >/dev/null
    command -v rosservice >/dev/null
    rospack find sim2real_planning_msgs >/dev/null
    python3 -c '\''from mavros_msgs.msg import State; from sim2real_planning_msgs.msg import PlannerGoal, PlannerStatus; from sim2real_planning_msgs.srv import ValidateGoal; from std_srvs.srv import Trigger'\''
  ' || die "Ground-station ROS/RViz environment verification failed. Rebuild the image."
  info "Ground-station container verified: $CONTAINER_NAME"
}

create_container() {
  local -a docker_args
  docker_args=(
    -d
    --init
    --name "$CONTAINER_NAME"
    --network host
    --ipc host
    -e "DISPLAY=$DISPLAY_VALUE"
    -e "QT_X11_NO_MITSHM=1"
    -e "SIM2REAL_RUNTIME_MODE=ground_station"
    -v /etc/localtime:/etc/localtime:ro
  )

  if [ -d /tmp/.X11-unix ]; then
    docker_args+=(-v /tmp/.X11-unix:/tmp/.X11-unix)
  else
    warn "/tmp/.X11-unix is absent; RViz requires X11 forwarding or a later container recreation from a graphical session."
  fi
  if [ -f "$HOME/.Xauthority" ]; then
    docker_args+=(-v "$HOME/.Xauthority:/root/.Xauthority:ro")
  fi
  if [ -d /dev/dri ]; then
    docker_args+=(--device /dev/dri:/dev/dri)
  fi
  append_extra_args "$EXTRA_DOCKER_ARGS" docker_args

  info "Creating ground-station container: $CONTAINER_NAME"
  docker run "${docker_args[@]}" "$IMAGE_NAME" tail -f /dev/null >/dev/null
  verify_environment
}

run_container() {
  ensure_prereqs
  ensure_image
  if container_exists; then
    container_layout_current ||
      die "Ground-station container '$CONTAINER_NAME' is stale. Run '$0 recreate'."
    if container_running; then
      info "Ground-station container is already running: $CONTAINER_NAME"
    else
      info "Starting ground-station container: $CONTAINER_NAME"
      docker start "$CONTAINER_NAME" >/dev/null
    fi
    verify_environment
  else
    create_container
  fi
}

stop_container() {
  ensure_prereqs
  if container_running; then
    info "Stopping ground-station container: $CONTAINER_NAME"
    docker stop "$CONTAINER_NAME" >/dev/null
  else
    info "Ground-station container is not running: $CONTAINER_NAME"
  fi
}

remove_container() {
  ensure_prereqs
  if container_exists; then
    info "Removing disposable ground-station container: $CONTAINER_NAME"
    docker rm -f "$CONTAINER_NAME" >/dev/null
  else
    info "Ground-station container does not exist: $CONTAINER_NAME"
  fi
}

recreate_container() {
  ensure_prereqs
  ensure_image
  remove_container
  create_container
}

shell_container() {
  run_container
  exec docker exec -it "$CONTAINER_NAME" bash -lc '
    source /root/.bashrc
    exec bash
  '
}

status_container() {
  ensure_prereqs
  docker ps -a --filter "name=^/${CONTAINER_NAME}$" --format 'table {{.Names}}\t{{.Image}}\t{{.Status}}'
}

main() {
  local action="${1:-}"
  shift || true
  [ "$#" -eq 0 ] || {
    usage >&2
    exit 1
  }

  case "$action" in
    build) build_image ;;
    run) run_container ;;
    stop) stop_container ;;
    recreate) recreate_container ;;
    rm) remove_container ;;
    shell) shell_container ;;
    status) status_container ;;
    verify)
      ensure_prereqs
      verify_environment
      ;;
    *)
      usage >&2
      exit 1
      ;;
  esac
}

main "$@"
