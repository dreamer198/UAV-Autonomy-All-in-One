#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
SIM_SCRIPT="$SCRIPT_DIR/sim.sh"
CONTAINER_SCRIPT="$SCRIPT_DIR/sim_container.sh"

CONTAINER_NAME="${SIM_DEV_CONTAINER:-diff_planner_px4_sim}"
BAG_NAME="${OUTDOOR_SIM_BAG_NAME:-se3_test_20260723_151241_0.bag}"
BAG_HOST="${OUTDOOR_SIM_BAG_HOST:-$PROJECT_ROOT/runtime/simulation/flight_bags/$BAG_NAME}"
BAG_CONTAINER="${OUTDOOR_SIM_BAG_CONTAINER:-/root/simulation_runtime/flight_bags/$BAG_NAME}"
SCENE_NAME="${OUTDOOR_SIM_SCENE_NAME:-${BAG_NAME%.bag}}"
OUTPUT_HOST="${OUTDOOR_SIM_OUTPUT_HOST:-$PROJECT_ROOT/runtime/simulation/reconstructed/$SCENE_NAME}"
OUTPUT_CONTAINER="${OUTDOOR_SIM_OUTPUT_CONTAINER:-/root/simulation_runtime/reconstructed/$SCENE_NAME}"
GENERATOR_CONTAINER="/workspaces/sim2real_ws/src/sim2real_simulation/scripts/reconstruct_bag_world.py"

VOXEL_SIZE="${OUTDOOR_SIM_VOXEL_SIZE:-0.14}"
CLOUD_STRIDE="${OUTDOOR_SIM_CLOUD_STRIDE:-2}"
MIN_OBSERVATIONS="${OUTDOOR_SIM_MIN_OBSERVATIONS:-2}"
CORRIDOR_RADIUS="${OUTDOOR_SIM_CORRIDOR_RADIUS:-7.0}"
OBSTACLE_MIN_Z="${OUTDOOR_SIM_OBSTACLE_MIN_Z:-0.18}"
OBSTACLE_MAX_Z="${OUTDOOR_SIM_OBSTACLE_MAX_Z:-3.2}"
WIND_SPEED="${OUTDOOR_SIM_WIND_SPEED:-0.0}"
WIND_DIRECTION_X="${OUTDOOR_SIM_WIND_DIRECTION_X:-1.0}"
WIND_DIRECTION_Y="${OUTDOOR_SIM_WIND_DIRECTION_Y:-0.0}"
WIND_DIRECTION_Z="${OUTDOOR_SIM_WIND_DIRECTION_Z:-0.0}"
usage() {
  cat <<'EOF'
Usage: outdoor_bag_sim.sh {generate|start|stop|status|attach|shell|arm|land|goal}

  generate           Reconstruct the complete Gazebo world from the bag.
  start              Compatibility alias for the unified sim.sh scene command.
  Other actions      Compatibility aliases for the corresponding sim.sh action.

New simulations should use the unified entrypoint directly:
  ./launch/sim.sh --scene se3_test_20260723_151241_0 restart
  ./launch/sim.sh arm
  ./launch/sim.sh goal X Y Z [YAW]
  ./launch/sim.sh land

The script intentionally does not replay recorded goals, modes or collision
events. It provides a repeatable environment and the recorded vehicle start.

Optional geometry/environment overrides:
  OUTDOOR_SIM_VOXEL_SIZE=0.14
  OUTDOOR_SIM_MIN_OBSERVATIONS=2
  OUTDOOR_SIM_CORRIDOR_RADIUS=7.0
  OUTDOOR_SIM_WIND_SPEED=0.0
  OUTDOOR_SIM_WIND_DIRECTION_X=1 OUTDOOR_SIM_WIND_DIRECTION_Y=0
  SIM_GAZEBO_GUI=false SIM_START_RVIZ=false
EOF
}

die() {
  echo "[ERROR] $*" >&2
  exit 1
}

ensure_container() {
  "$CONTAINER_SCRIPT" run
}

generate_scene() {
  [ -f "$BAG_HOST" ] || die "Repaired bag not found: $BAG_HOST"
  ensure_container
  mkdir -p "$OUTPUT_HOST"
  echo "[INFO] Reconstructing the full recorded flight corridor."
  docker exec -i \
    "$CONTAINER_NAME" bash -lc "
      set -eo pipefail
      source /root/.bashrc
      python3 '$GENERATOR_CONTAINER' \
        '$BAG_CONTAINER' '$OUTPUT_CONTAINER' \
        --voxel-size '$VOXEL_SIZE' \
        --cloud-stride '$CLOUD_STRIDE' \
        --min-observations '$MIN_OBSERVATIONS' \
        --corridor-radius '$CORRIDOR_RADIUS' \
        --obstacle-min-z '$OBSTACLE_MIN_Z' \
        --obstacle-max-z '$OBSTACLE_MAX_Z' \
        --wind-speed '$WIND_SPEED' \
        --wind-direction '$WIND_DIRECTION_X' '$WIND_DIRECTION_Y' '$WIND_DIRECTION_Z'
    "
}

ensure_scene() {
  if [ ! -f "$OUTPUT_HOST/se3_outdoor_reconstruction.world" ] || \
     [ ! -f "$OUTPUT_HOST/meshes/scene.obj" ] || \
     [ ! -f "$OUTPUT_HOST/metadata.json" ]; then
    generate_scene
  fi
}

sim_command() {
  ensure_scene
  env \
    SIM_DEV_CONTAINER="$CONTAINER_NAME" \
    "$SIM_SCRIPT" --scene "$SCENE_NAME" "$@"
}

main() {
  local action="${1:-}"
  shift || true
  case "$action" in
    generate)
      generate_scene
      ;;
    start)
      sim_command restart
      ;;
    stop|status|attach|shell|arm|land|goal)
      sim_command "$action" "$@"
      ;;
    *)
      usage
      exit 1
      ;;
  esac
}

main "$@"
