#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
MANIFEST_ROOT="$PROJECT_ROOT/planning/plugins"
MANAGER_ROOT="$PROJECT_ROOT/planning/ros_pkgs/sim2real_planner_manager"

fail() {
  echo "[ERROR] $*" >&2
  exit 1
}

for script in \
  "$PROJECT_ROOT/launch/sim.sh" \
  "$PROJECT_ROOT/launch/real.sh" \
  "$PROJECT_ROOT/launch/sim_container.sh" \
  "$PROJECT_ROOT/launch/real_container.sh" \
  "$SCRIPT_DIR/build_planner_workspaces.sh"; do
  bash -n "$script"
done

if command -v shellcheck >/dev/null 2>&1; then
  shellcheck \
    "$PROJECT_ROOT/launch/sim.sh" \
    "$PROJECT_ROOT/launch/real.sh" \
    "$PROJECT_ROOT/launch/sim_container.sh" \
    "$PROJECT_ROOT/launch/real_container.sh" \
    "$SCRIPT_DIR/build_planner_workspaces.sh"
fi

python3 "$SCRIPT_DIR/planner_manifest.py" \
  --project-root "$PROJECT_ROOT" \
  --manifest-root "$MANIFEST_ROOT" \
  list >/dev/null

for planner in diff fast-kino fast-topo; do
  python3 "$SCRIPT_DIR/planner_manifest.py" \
    --project-root "$PROJECT_ROOT" \
    --manifest-root "$MANIFEST_ROOT" \
    resolve "$planner" --mode simulation >/dev/null
done

for planner in fast-kino fast-topo; do
  if python3 "$SCRIPT_DIR/planner_manifest.py" \
    --project-root "$PROJECT_ROOT" \
    --manifest-root "$MANIFEST_ROOT" \
    resolve "$planner" --mode simulation --profile outdoor >/dev/null 2>&1; then
    fail "$planner must expose only the unified built-in map configuration"
  fi
done
[ ! -e "$PROJECT_ROOT/planning/ros_pkgs/sim2real_fast_adapter/config/outdoor.yaml" ] ||
  fail "legacy Fast outdoor map configuration must not be present"
[ -f "$PROJECT_ROOT/planning/ros_pkgs/sim2real_fast_adapter/config/planner.yaml" ] ||
  fail "Fast core planner configuration is missing"
[ -f "$PROJECT_ROOT/planning/ros_pkgs/sim2real_fast_adapter/config/scenes/forest.yaml" ] ||
  fail "Fast forest scene map configuration is missing"
for legacy_config in base local forest; do
  [ ! -e "$PROJECT_ROOT/planning/ros_pkgs/sim2real_fast_adapter/config/$legacy_config.yaml" ] ||
    fail "legacy Fast configuration '$legacy_config.yaml' must not remain"
done
[ -f "$PROJECT_ROOT/planning/ros_pkgs/sim2real_diff_adapter/config/planner.yaml" ] ||
  fail "Diff unified planner configuration is missing"
for legacy_config in diff trajectory_server; do
  [ ! -e "$PROJECT_ROOT/planning/ros_pkgs/sim2real_diff_adapter/config/$legacy_config.yaml" ] ||
    fail "legacy Diff configuration '$legacy_config.yaml' must not remain"
done
for scene in room forest; do
  [ -f "$PROJECT_ROOT/simulation/config/scenes/$scene.env" ] ||
    fail "simulation scene '$scene' is missing"
done
[ -d "$PROJECT_ROOT/simulation/config/scenes/forest" ] ||
  fail "forest scene assets are missing"
for legacy_scene in default outdoor_rectangular_forest; do
  [ ! -e "$PROJECT_ROOT/simulation/config/scenes/$legacy_scene.env" ] ||
    fail "legacy simulation scene '$legacy_scene' must not remain"
  [ ! -e "$PROJECT_ROOT/simulation/config/scenes/$legacy_scene" ] ||
    fail "legacy simulation scene assets '$legacy_scene' must not remain"
done
grep -q 'runtime_mode:=simulation scene:=%q' "$PROJECT_ROOT/launch/sim.sh" ||
  fail "simulation launcher does not pass its validated scene to the planner"
grep -q -- '--arg scene=$(arg scene)' \
  "$MANAGER_ROOT/launch/planner_gateway.launch" ||
  fail "planner manager does not forward scene context to the selected plugin"

for planner in diff fast-kino fast-topo; do
  python3 "$SCRIPT_DIR/planner_manifest.py" \
    --project-root "$PROJECT_ROOT" \
    --manifest-root "$MANIFEST_ROOT" \
    resolve "$planner" --mode real >/dev/null
done

PYTHONPATH="$MANAGER_ROOT/src${PYTHONPATH:+:$PYTHONPATH}" \
  python3 "$MANAGER_ROOT/scripts/planner_plugins.py" \
    --manifest-root "$MANIFEST_ROOT" \
    --repository-root "$PROJECT_ROOT" \
    validate diff fast-kino fast-topo >/dev/null

grep -q -- '--planner-profile' "$PROJECT_ROOT/launch/sim.sh" ||
  fail "simulation launcher does not expose --planner-profile"
grep -q -- '--planner-profile' "$PROJECT_ROOT/launch/real.sh" ||
  fail "real launcher does not expose --planner-profile"
grep -Fq 'PLANNER_ID="${SIM_PLANNER:-}"' "$PROJECT_ROOT/launch/sim.sh" ||
  fail "simulation launcher must not choose a default planner"
grep -Fq 'SCENE="${SIM_SCENE:-}"' "$PROJECT_ROOT/launch/sim.sh" ||
  fail "simulation launcher must not choose a default scene"
grep -Fq 'PLANNER_ID="${REAL_PLANNER:-}"' "$PROJECT_ROOT/launch/real.sh" ||
  fail "real launcher must not choose a default planner"
grep -q 'require_planner_selection' "$PROJECT_ROOT/launch/sim.sh" ||
  fail "simulation launcher does not require an explicit planner selection"
grep -q 'require_scene_selection' "$PROJECT_ROOT/launch/sim.sh" ||
  fail "simulation launcher does not require an explicit scene selection"
grep -q 'require_planner_selection' "$PROJECT_ROOT/launch/real.sh" ||
  fail "real launcher does not require an explicit planner selection"
grep -q '/planning/status' "$PROJECT_ROOT/launch/sim.sh" ||
  fail "simulation rosbag/status contract is missing /planning/status"
grep -q '/planning/status' "$PROJECT_ROOT/launch/real.sh" ||
  fail "real rosbag/status contract is missing /planning/status"
grep -q 'name="planner_visualization"' \
  "$MANAGER_ROOT/launch/planner_gateway.launch" ||
  fail "planner manager does not own the common visualization bridge"
for backend_launch in \
  "$PROJECT_ROOT/planning/ros_pkgs/sim2real_diff_adapter/launch/diff_backend.launch" \
  "$PROJECT_ROOT/planning/ros_pkgs/sim2real_fast_adapter/launch/fast_backend.launch"; do
  grep -q '/planning/viz/raw/occupancy' "$backend_launch" ||
    fail "$backend_launch does not publish raw occupancy for normalization"
  grep -q '/planning/viz/raw/inflated_occupancy' "$backend_launch" ||
    fail "$backend_launch does not publish raw inflation for normalization"
  if grep -Eq 'to="/planning/viz/(occupancy|inflated_occupancy)"' \
    "$backend_launch"; then
    fail "$backend_launch bypasses the common visualization bridge"
  fi
done
grep -q 'NLOPT_DEBIAN_VERSION=2.6.1-8ubuntu2' "$PROJECT_ROOT/simulation/Dockerfile" ||
  fail "simulation image is missing the NLopt C++ development package"
grep -q 'NLOPT_DEBIAN_VERSION=2.6.1-8ubuntu2' "$PROJECT_ROOT/deployment/Dockerfile" ||
  fail "real image is missing the NLopt C++ development package"

python3 - \
  "$PROJECT_ROOT/common/launch/planner.launch" \
  "$PROJECT_ROOT/common/launch/planning_control.launch" \
  "$MANAGER_ROOT/launch/planner_gateway.launch" <<'PY'
import sys
import xml.etree.ElementTree as ET

for path in sys.argv[1:]:
    planner_args = [
        element
        for element in ET.parse(path).getroot().findall("arg")
        if element.attrib.get("name") == "planner_id"
    ]
    if len(planner_args) != 1 or "default" in planner_args[0].attrib:
        raise SystemExit(
            "{} must require planner_id without a default".format(path)
        )
PY

python3 - \
  "$PROJECT_ROOT/third_party/Fast-Planner/fast_planner/plan_manage/src/topo_replan_fsm.cpp" <<'PY'
import pathlib
import sys

source = pathlib.Path(sys.argv[1]).read_text(encoding="utf-8")
replan_new = source.split("case REPLAN_NEW:", 1)[1].split("case ", 1)[0]
truncate = replan_new.find("replan_pub_.publish")
blocking_plan = replan_new.find("callTopologicalTraj(1)")
if truncate < 0 or blocking_plan < 0 or truncate > blocking_plan:
    raise SystemExit(
        "Fast Topo REPLAN_NEW must truncate the old trajectory before planning"
    )
PY

echo "[INFO] Planner manifest, launcher, capability, and image checks passed."
