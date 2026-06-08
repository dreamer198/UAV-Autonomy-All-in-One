#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE_NAME="${IMAGE_NAME:-ros_noetic_realflight:latest}"
CONTAINER_NAME="${CONTAINER_NAME:-ros_noetic_realflight}"
FCU_DEVICE="${FCU_DEVICE:-/dev/ttyACM0}"
DISPLAY_VALUE="${DISPLAY:-:0}"
DRONE_ID="${DRONE_ID:-0}"
FCU_URL="${FCU_URL:-}"
GCS_URL="${GCS_URL:-}"
RUNTIME_DIR="${RUNTIME_DIR:-$PROJECT_ROOT/runtime}"
EXTRA_DOCKER_ARGS="${EXTRA_DOCKER_ARGS:-}"

usage() {
  cat <<'EOF'
Usage: docker_run_real.sh {build|run|stop|rm|restart|shell|status}
EOF
}

need_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "[ERROR] Missing command: $1"
    exit 1
  fi
}

ensure_prereqs() {
  need_cmd docker
}

container_exists() {
  docker inspect "$CONTAINER_NAME" >/dev/null 2>&1
}

container_running() {
  [ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER_NAME" 2>/dev/null || echo false)" = "true" ]
}

build_image() {
  ensure_prereqs
  echo "[INFO] Building image: $IMAGE_NAME"
  docker build -t "$IMAGE_NAME" -f "$PROJECT_ROOT/docker/Dockerfile" "$PROJECT_ROOT"
}

remove_container() {
  if container_exists; then
    echo "[INFO] Removing container: $CONTAINER_NAME"
    if ! docker rm -f "$CONTAINER_NAME" >/dev/null 2>&1; then
      if container_exists; then
        echo "[ERROR] Failed to remove container: $CONTAINER_NAME" >&2
        return 1
      fi
    fi
  else
    echo "[INFO] Container does not exist: $CONTAINER_NAME"
  fi
}

run_container() {
  ensure_prereqs

  mkdir -p "$RUNTIME_DIR/flight_bags" "$RUNTIME_DIR/tmp"

  if ! docker image inspect "$IMAGE_NAME" >/dev/null 2>&1; then
    build_image
  fi

  if container_exists; then
    remove_container
  fi

  local -a docker_args
  docker_args=(
    -d
    --name "$CONTAINER_NAME"
    --network host
    --ipc host
    --privileged
    -e "DRONE_ID=$DRONE_ID"
    -e "DISPLAY=$DISPLAY_VALUE"
    -e "QT_X11_NO_MITSHM=1"
    -v /etc/localtime:/etc/localtime:ro
    -v "$RUNTIME_DIR/flight_bags:/root/flight_bags"
    -v "$RUNTIME_DIR/tmp:/root/tmp"
    -v "$PROJECT_ROOT/config/livox/MID360s_config.json:/root/livox_ws/src/livox_ros_driver2/config/MID360s_config.json:ro"
    -v "$PROJECT_ROOT/scripts/start_real_px4_mid360_fastlio.sh:/root/code/start_real_px4_mid360_fastlio.sh:ro"
  )

  if [ -n "$FCU_URL" ]; then
    docker_args+=(-e "FCU_URL=$FCU_URL")
  fi

  if [ -n "$GCS_URL" ]; then
    docker_args+=(-e "GCS_URL=$GCS_URL")
  fi

  if [ -e "$FCU_DEVICE" ]; then
    docker_args+=(--device "$FCU_DEVICE:$FCU_DEVICE")
  else
    echo "[WARN] FCU device not found on host: $FCU_DEVICE"
  fi

  if [ -d /tmp/.X11-unix ]; then
    docker_args+=(-v /tmp/.X11-unix:/tmp/.X11-unix)
  fi

  if [ -f "$HOME/.Xauthority" ]; then
    docker_args+=(-v "$HOME/.Xauthority:/root/.Xauthority:ro")
  fi

  if [ -n "$EXTRA_DOCKER_ARGS" ]; then
    # shellcheck disable=SC2206
    docker_args+=($EXTRA_DOCKER_ARGS)
  fi

  echo "[INFO] Starting container: $CONTAINER_NAME"
  docker run "${docker_args[@]}" "$IMAGE_NAME" tail -f /dev/null
}

stop_container() {
  ensure_prereqs
  if container_running; then
    echo "[INFO] Stopping container: $CONTAINER_NAME"
    docker stop "$CONTAINER_NAME" >/dev/null
  else
    echo "[INFO] Container is not running: $CONTAINER_NAME"
  fi
}

shell_container() {
  ensure_prereqs
  if ! container_running; then
    echo "[ERROR] Container is not running: $CONTAINER_NAME"
    exit 1
  fi
  exec docker exec -it "$CONTAINER_NAME" bash
}

status_container() {
  ensure_prereqs
  docker ps -a --filter "name=^/${CONTAINER_NAME}$" --format 'table {{.Names}}\t{{.Image}}\t{{.Status}}'
}

main() {
  local action="${1:-}"

  case "$action" in
    build)
      build_image
      ;;
    run)
      run_container
      ;;
    stop)
      stop_container
      ;;
    rm)
      remove_container
      ;;
    restart)
      run_container
      ;;
    shell)
      shell_container
      ;;
    status)
      status_container
      ;;
    *)
      usage
      exit 1
      ;;
  esac
}

main "$@"
