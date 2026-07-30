#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
SIM_SCRIPT="$SCRIPT_DIR/sim.sh"
CONTAINER_SCRIPT="$SCRIPT_DIR/sim_container.sh"

CONTAINER_NAME="${SIM_DEV_CONTAINER:-uav_autonomy_sim}"
BAG_NAME="${OUTDOOR_SIM_BAG_NAME:-se3_test_20260723_151241_0.bag}"
BAG_HOST="${OUTDOOR_SIM_BAG_HOST:-$PROJECT_ROOT/runtime/simulation/flight_bags/$BAG_NAME}"
BAG_CONTAINER="${OUTDOOR_SIM_BAG_CONTAINER:-/root/simulation_runtime/flight_bags/$BAG_NAME}"
SCENE_NAME="${OUTDOOR_SIM_SCENE_NAME:-forest}"
OUTPUT_ROOT="$PROJECT_ROOT/runtime/simulation/reconstructed"
OUTPUT_HOST="${OUTDOOR_SIM_OUTPUT_HOST:-$OUTPUT_ROOT/$SCENE_NAME}"
OUTPUT_CONTAINER="${OUTDOOR_SIM_OUTPUT_CONTAINER:-/root/simulation_runtime/reconstructed/$SCENE_NAME}"
ASSET_ROOT="$PROJECT_ROOT/simulation/config/scenes"
ASSET_HOST="${OUTDOOR_SIM_ASSET_HOST:-$ASSET_ROOT/$SCENE_NAME}"
ASSET_CONTAINER="${OUTDOOR_SIM_ASSET_CONTAINER:-/etc/sim2real/simulation/scenes/$SCENE_NAME}"
GENERATOR_CONTAINER="/workspaces/sim2real_ws/src/sim2real_simulation/scripts/reconstruct_bag_world.py"

VOXEL_SIZE="${OUTDOOR_SIM_VOXEL_SIZE:-0.14}"
CLOUD_STRIDE="${OUTDOOR_SIM_CLOUD_STRIDE:-2}"
MIN_OBSERVATIONS="${OUTDOOR_SIM_MIN_OBSERVATIONS:-2}"
CORRIDOR_RADIUS="${OUTDOOR_SIM_CORRIDOR_RADIUS:-7.0}"
OBSTACLE_MIN_Z="${OUTDOOR_SIM_OBSTACLE_MIN_Z:-0.18}"
OBSTACLE_MAX_Z="${OUTDOOR_SIM_OBSTACLE_MAX_Z:-3.2}"
GEOMETRY_MODE="${OUTDOOR_SIM_GEOMETRY_MODE:-forest}"
TREE_GRID_SIZE="${OUTDOOR_SIM_TREE_GRID_SIZE:-0.28}"
TREE_SMOOTHING_RADIUS="${OUTDOOR_SIM_TREE_SMOOTHING_RADIUS:-0.40}"
TREE_MIN_SPACING="${OUTDOOR_SIM_TREE_MIN_SPACING:-1.20}"
TREE_DENSITY_QUANTILE="${OUTDOOR_SIM_TREE_DENSITY_QUANTILE:-0.80}"
TREE_TRUNK_RADIUS="${OUTDOOR_SIM_TREE_TRUNK_RADIUS:-0.19}"
TREE_CROWN_RADIUS="${OUTDOOR_SIM_TREE_CROWN_RADIUS:-0.78}"
TREE_MIN_HEIGHT="${OUTDOOR_SIM_TREE_MIN_HEIGHT:-2.50}"
TREE_MAX_HEIGHT="${OUTDOOR_SIM_TREE_MAX_HEIGHT:-4.20}"
FOREST_MIN_X="${OUTDOOR_SIM_FOREST_MIN_X:--0.120791277208}"
FOREST_MAX_X="${OUTDOOR_SIM_FOREST_MAX_X:-61.65}"
FOREST_MIN_Y="${OUTDOOR_SIM_FOREST_MIN_Y:--19.03}"
FOREST_MAX_Y="${OUTDOOR_SIM_FOREST_MAX_Y:--0.0364426606833}"
FOREST_FILL_SPACING="${OUTDOOR_SIM_FOREST_FILL_SPACING:-2.0}"
FOREST_PATH_CLEARANCE="${OUTDOOR_SIM_FOREST_PATH_CLEARANCE:-0.65}"
FOREST_CORNER_CLEARANCE="${OUTDOOR_SIM_FOREST_CORNER_CLEARANCE:-1.50}"
FOREST_SEED="${OUTDOOR_SIM_FOREST_SEED:-151241}"
WIND_SPEED="${OUTDOOR_SIM_WIND_SPEED:-0.0}"
WIND_DIRECTION_X="${OUTDOOR_SIM_WIND_DIRECTION_X:-1.0}"
WIND_DIRECTION_Y="${OUTDOOR_SIM_WIND_DIRECTION_Y:-0.0}"
WIND_DIRECTION_Z="${OUTDOOR_SIM_WIND_DIRECTION_Z:-0.0}"
usage() {
  cat <<'EOF'
Usage: outdoor_bag_sim.sh {generate|start|stop|status|attach|shell|arm|land|goal}

  generate           Reconstruct and publish the versioned Gazebo forest asset.
  start              Compatibility alias for the unified sim.sh scene command.
  Other actions      Compatibility aliases for the corresponding sim.sh action.

New simulations should use the unified entrypoint directly:
  ./launch/sim.sh --scene forest --planner diff restart
  ./launch/sim.sh arm
  ./launch/sim.sh goal X Y Z [YAW]
  ./launch/sim.sh land

The script intentionally does not replay recorded goals, modes or collision
events. It provides a repeatable environment and the recorded vehicle start.

Optional geometry/environment overrides:
  OUTDOOR_SIM_VOXEL_SIZE=0.14
  OUTDOOR_SIM_MIN_OBSERVATIONS=2
  OUTDOOR_SIM_CORRIDOR_RADIUS=7.0
  OUTDOOR_SIM_TREE_MIN_SPACING=1.20
  OUTDOOR_SIM_TREE_DENSITY_QUANTILE=0.80
  OUTDOOR_SIM_TREE_CROWN_RADIUS=0.78
  OUTDOOR_SIM_TREE_MAX_HEIGHT=4.20
  OUTDOOR_SIM_FOREST_FILL_SPACING=2.0
  OUTDOOR_SIM_FOREST_PATH_CLEARANCE=0.65
  OUTDOOR_SIM_GEOMETRY_MODE=forest
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

validate_publish_paths() {
  command -v realpath >/dev/null 2>&1 || die "Missing host command: realpath"
  [[ "$SCENE_NAME" =~ ^[A-Za-z0-9][A-Za-z0-9_-]*$ ]] ||
    die "OUTDOOR_SIM_SCENE_NAME must contain only letters, digits, '_' and '-'."
  local output_root_real asset_root_real output_real asset_real
  local output_container_real asset_container_real
  output_root_real="$(realpath -m "$OUTPUT_ROOT")"
  asset_root_real="$(realpath -m "$ASSET_ROOT")"
  output_real="$(realpath -m "$OUTPUT_HOST")"
  asset_real="$(realpath -m "$ASSET_HOST")"
  output_container_real="$(realpath -m "$OUTPUT_CONTAINER")"
  asset_container_real="$(realpath -m "$ASSET_CONTAINER")"
  case "$output_real" in
    "$output_root_real"/*) ;;
    *) die "OUTDOOR_SIM_OUTPUT_HOST must stay below $output_root_real (got: $output_real)" ;;
  esac
  case "$asset_real" in
    "$asset_root_real"/*) ;;
    *) die "OUTDOOR_SIM_ASSET_HOST must stay below $asset_root_real (got: $asset_real)" ;;
  esac
  case "$output_container_real" in
    /root/simulation_runtime/reconstructed/*) ;;
    *) die "OUTDOOR_SIM_OUTPUT_CONTAINER must stay below /root/simulation_runtime/reconstructed." ;;
  esac
  case "$asset_container_real" in
    /etc/sim2real/simulation/scenes/*) ;;
    *) die "OUTDOOR_SIM_ASSET_CONTAINER must stay below /etc/sim2real/simulation/scenes." ;;
  esac
  case "$output_real/" in
    "$asset_real/"*|"$asset_root_real/"*) die "Reconstruction output and scene asset paths must not overlap." ;;
  esac
  case "$asset_real/" in
    "$output_real/"*|"$output_root_real/"*) die "Reconstruction output and scene asset paths must not overlap." ;;
  esac
  OUTPUT_HOST="$output_real"
  ASSET_HOST="$asset_real"
  OUTPUT_CONTAINER="$output_container_real"
  ASSET_CONTAINER="$asset_container_real"
}

generate_scene() {
  validate_publish_paths
  [ -f "$BAG_HOST" ] || die "Repaired bag not found: $BAG_HOST"
  command -v rsync >/dev/null 2>&1 ||
    die "Missing host command required to publish the scene: rsync"
  ensure_container
  mkdir -p "$OUTPUT_HOST"
  echo "[INFO] Reconstructing the full recorded flight corridor."
  docker exec -i \
    -e "BAG_CONTAINER=$BAG_CONTAINER" \
    -e "OUTPUT_CONTAINER=$OUTPUT_CONTAINER" \
    -e "GENERATOR_CONTAINER=$GENERATOR_CONTAINER" \
    -e "VOXEL_SIZE=$VOXEL_SIZE" \
    -e "CLOUD_STRIDE=$CLOUD_STRIDE" \
    -e "MIN_OBSERVATIONS=$MIN_OBSERVATIONS" \
    -e "CORRIDOR_RADIUS=$CORRIDOR_RADIUS" \
    -e "OBSTACLE_MIN_Z=$OBSTACLE_MIN_Z" \
    -e "OBSTACLE_MAX_Z=$OBSTACLE_MAX_Z" \
    -e "GEOMETRY_MODE=$GEOMETRY_MODE" \
    -e "TREE_GRID_SIZE=$TREE_GRID_SIZE" \
    -e "TREE_SMOOTHING_RADIUS=$TREE_SMOOTHING_RADIUS" \
    -e "TREE_MIN_SPACING=$TREE_MIN_SPACING" \
    -e "TREE_DENSITY_QUANTILE=$TREE_DENSITY_QUANTILE" \
    -e "TREE_TRUNK_RADIUS=$TREE_TRUNK_RADIUS" \
    -e "TREE_CROWN_RADIUS=$TREE_CROWN_RADIUS" \
    -e "TREE_MIN_HEIGHT=$TREE_MIN_HEIGHT" \
    -e "TREE_MAX_HEIGHT=$TREE_MAX_HEIGHT" \
    -e "FOREST_MIN_X=$FOREST_MIN_X" \
    -e "FOREST_MAX_X=$FOREST_MAX_X" \
    -e "FOREST_MIN_Y=$FOREST_MIN_Y" \
    -e "FOREST_MAX_Y=$FOREST_MAX_Y" \
    -e "FOREST_FILL_SPACING=$FOREST_FILL_SPACING" \
    -e "FOREST_PATH_CLEARANCE=$FOREST_PATH_CLEARANCE" \
    -e "FOREST_CORNER_CLEARANCE=$FOREST_CORNER_CLEARANCE" \
    -e "FOREST_SEED=$FOREST_SEED" \
    -e "ASSET_CONTAINER=$ASSET_CONTAINER" \
    -e "WIND_SPEED=$WIND_SPEED" \
    -e "WIND_DIRECTION_X=$WIND_DIRECTION_X" \
    -e "WIND_DIRECTION_Y=$WIND_DIRECTION_Y" \
    -e "WIND_DIRECTION_Z=$WIND_DIRECTION_Z" \
    "$CONTAINER_NAME" bash -lc '
      set -eo pipefail
      source /root/.bashrc
      python3 "$GENERATOR_CONTAINER" \
        "$BAG_CONTAINER" "$OUTPUT_CONTAINER" \
        --voxel-size "$VOXEL_SIZE" \
        --cloud-stride "$CLOUD_STRIDE" \
        --min-observations "$MIN_OBSERVATIONS" \
        --corridor-radius "$CORRIDOR_RADIUS" \
        --obstacle-min-z "$OBSTACLE_MIN_Z" \
        --obstacle-max-z "$OBSTACLE_MAX_Z" \
        --geometry-mode "$GEOMETRY_MODE" \
        --tree-grid-size "$TREE_GRID_SIZE" \
        --tree-smoothing-radius "$TREE_SMOOTHING_RADIUS" \
        --tree-min-spacing "$TREE_MIN_SPACING" \
        --tree-density-quantile "$TREE_DENSITY_QUANTILE" \
        --tree-trunk-radius "$TREE_TRUNK_RADIUS" \
        --tree-crown-radius "$TREE_CROWN_RADIUS" \
        --tree-min-height "$TREE_MIN_HEIGHT" \
        --tree-max-height "$TREE_MAX_HEIGHT" \
        --forest-min-x "$FOREST_MIN_X" \
        --forest-max-x "$FOREST_MAX_X" \
        --forest-min-y "$FOREST_MIN_Y" \
        --forest-max-y "$FOREST_MAX_Y" \
        --forest-fill-spacing "$FOREST_FILL_SPACING" \
        --forest-path-clearance "$FOREST_PATH_CLEARANCE" \
        --forest-corner-clearance "$FOREST_CORNER_CLEARANCE" \
        --forest-seed "$FOREST_SEED" \
        --mesh-uri "file://$ASSET_CONTAINER/meshes/scene.obj" \
        --wind-speed "$WIND_SPEED" \
        --wind-direction "$WIND_DIRECTION_X" "$WIND_DIRECTION_Y" "$WIND_DIRECTION_Z"
    '
  if [ ! -f "$OUTPUT_HOST/se3_outdoor_reconstruction.world" ] ||
    [ ! -f "$OUTPUT_HOST/meshes/scene.obj" ] ||
    [ ! -f "$OUTPUT_HOST/metadata.json" ]; then
    die "Scene generator did not produce the expected world, mesh and metadata files."
  fi
  mkdir -p "$ASSET_HOST"
  rsync -a --delete -- "$OUTPUT_HOST/" "$ASSET_HOST/"
  echo "[INFO] Published versioned scene asset: $ASSET_HOST"
}

ensure_scene() {
  validate_publish_paths
  if [ ! -f "$ASSET_HOST/se3_outdoor_reconstruction.world" ] || \
     [ ! -f "$ASSET_HOST/meshes/scene.obj" ] || \
     [ ! -f "$ASSET_HOST/metadata.json" ]; then
    generate_scene
  fi
}

sim_command() {
  if [ -n "${SIM_DEV_CONTAINER:-}" ]; then
    env \
      SIM_DEV_CONTAINER="$SIM_DEV_CONTAINER" \
      "$SIM_SCRIPT" --scene "$SCENE_NAME" "$@"
  else
    "$SIM_SCRIPT" --scene "$SCENE_NAME" "$@"
  fi
}

main() {
  local action="${1:-}"
  shift || true
  case "$action" in
    generate)
      generate_scene
      ;;
    start)
      ensure_scene
      sim_command restart
      ;;
    shell)
      sim_command shell "$@"
      ;;
    stop|status|attach|arm|land|goal)
      sim_command "$action" "$@"
      ;;
    *)
      usage
      exit 1
      ;;
  esac
}

main "$@"
