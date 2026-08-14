#!/usr/bin/env bash
# shellcheck shell=bash disable=SC2016
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
  "$PROJECT_ROOT/launch/real_bag.sh" \
  "$PROJECT_ROOT/launch/sim_container.sh" \
  "$PROJECT_ROOT/launch/real_container.sh" \
  "$SCRIPT_DIR/build_planner_workspaces.sh"; do
  bash -n "$script"
done

if command -v shellcheck >/dev/null 2>&1; then
  shellcheck \
    "$PROJECT_ROOT/launch/sim.sh" \
    "$PROJECT_ROOT/launch/real.sh" \
    "$PROJECT_ROOT/launch/real_bag.sh" \
    "$PROJECT_ROOT/launch/sim_container.sh" \
    "$PROJECT_ROOT/launch/real_container.sh" \
    "$SCRIPT_DIR/build_planner_workspaces.sh" \
    "$SCRIPT_DIR/check_planner_integration.sh"
fi

python3 "$SCRIPT_DIR/planner_manifest.py" \
  --project-root "$PROJECT_ROOT" \
  --manifest-root "$MANIFEST_ROOT" \
  list >/dev/null

builtin_ids="$(
  SIM2REAL_PLANNER_PLUGIN_PATH='' \
    python3 "$SCRIPT_DIR/planner_manifest.py" \
    --project-root "$PROJECT_ROOT" \
    --manifest-root "$MANIFEST_ROOT" \
    list --json |
    python3 -c 'import json, sys; print(" ".join(item["id"] for item in json.load(sys.stdin)))'
)"
[ "$builtin_ids" = "diff fast-kino fast-topo super" ] ||
  fail "built-in planner set must be exactly: diff fast-kino fast-topo super (got: $builtin_ids)"

for planner in diff fast-kino fast-topo super; do
  python3 "$SCRIPT_DIR/planner_manifest.py" \
    --project-root "$PROJECT_ROOT" \
    --manifest-root "$MANIFEST_ROOT" \
    resolve "$planner" --mode simulation >/dev/null
done

for planner in fast-kino fast-topo super; do
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
[ -f "$PROJECT_ROOT/planning/ros_pkgs/sim2real_super_adapter/config/planner.yaml" ] ||
  fail "SUPER unified planner configuration is missing"
[ -f "$PROJECT_ROOT/planning/ros_pkgs/sim2real_super_adapter/launch/super_backend.launch" ] ||
  fail "SUPER backend launch is missing"
[ -f "$PROJECT_ROOT/third_party/SUPER/NOTICE.md" ] ||
  fail "SUPER provenance and license notice is missing"
grep -Fq '2ad3419c127a617c6d7df6925e81a14175a9c096' \
  "$PROJECT_ROOT/third_party/SUPER/NOTICE.md" ||
  fail "SUPER notice does not pin the selected upstream commit"
for package in \
  super_planner \
  rog_map \
  mars_uav_sim/mars_quadrotor_msgs; do
  [ -f "$PROJECT_ROOT/third_party/SUPER/$package/package.xml" ] ||
    fail "selected SUPER package is missing: $package"
done
for excluded in \
  mission_planner \
  mars_uav_sim/perfect_drone_sim \
  mars_uav_sim/marsim_render; do
  [ ! -e "$PROJECT_ROOT/third_party/SUPER/$excluded" ] ||
    fail "non-runtime SUPER package must not be vendored: $excluded"
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

for planner in diff fast-kino fast-topo super; do
  python3 "$SCRIPT_DIR/planner_manifest.py" \
    --project-root "$PROJECT_ROOT" \
    --manifest-root "$MANIFEST_ROOT" \
    resolve "$planner" --mode real >/dev/null
done
grep -Fq '(runtime_mode_ != "simulation" && runtime_mode_ != "real")' \
  "$PROJECT_ROOT/planning/ros_pkgs/sim2real_fast_adapter/src/fast_backend_adapter_node.cpp" ||
  fail "Fast adapter does not accept both common runtime modes"

PYTHONPATH="$MANAGER_ROOT/src${PYTHONPATH:+:$PYTHONPATH}" \
  python3 "$MANAGER_ROOT/scripts/planner_plugins.py" \
    --manifest-root "$MANIFEST_ROOT" \
    --repository-root "$PROJECT_ROOT" \
    validate diff fast-kino fast-topo super >/dev/null

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
for launcher in \
  "$PROJECT_ROOT/launch/sim.sh" \
  "$PROJECT_ROOT/launch/real.sh"; do
  grep -q 'SIM2REAL_PLANNER_CONFIG' "$launcher" ||
    fail "$launcher does not forward the common planner configuration override"
  if grep -q 'SIM2REAL_DIFF_PLANNER_CONFIG' "$launcher"; then
    fail "$launcher retains a Diff-only planner configuration override"
  fi
done
if grep -Fq 'if [ "$PLANNER_ID" = "diff" ]' \
  "$PROJECT_ROOT/launch/real.sh"; then
  fail "real launcher must not branch on a specific planner"
fi
for backend_launch in \
  "$PROJECT_ROOT/planning/ros_pkgs/sim2real_diff_adapter/launch/diff_backend.launch" \
  "$PROJECT_ROOT/planning/ros_pkgs/sim2real_fast_adapter/launch/fast_backend.launch" \
  "$PROJECT_ROOT/planning/ros_pkgs/sim2real_super_adapter/launch/super_backend.launch"; do
  grep -q 'SIM2REAL_PLANNER_CONFIG' "$backend_launch" ||
    fail "$backend_launch does not accept the common configuration override"
  grep -q '/planning/viz/raw/occupancy' "$backend_launch" ||
    fail "$backend_launch does not publish raw occupancy for normalization"
  grep -q '/planning/viz/raw/inflated_occupancy' "$backend_launch" ||
    fail "$backend_launch does not publish raw inflation for normalization"
  if grep -Eq 'to="/planning/viz/(occupancy|inflated_occupancy)"' \
    "$backend_launch"; then
    fail "$backend_launch bypasses the common visualization bridge"
  fi
done
if grep -q '/command/trajectory' \
  "$PROJECT_ROOT/planning/ros_pkgs/sim2real_super_adapter/launch/super_backend.launch"; then
  fail "SUPER backend must not publish the gateway-owned controller topic"
fi
grep -q '/planning/backends/super/native/' \
  "$PROJECT_ROOT/planning/ros_pkgs/sim2real_super_adapter/config/planner.yaml" ||
  fail "SUPER native interfaces are not scoped below the backend native namespace"
for native_visualization in \
  visualization/replan_log_mkr \
  visualization/replan_log_pc \
  rog_map/esdf/neg \
  rog_map/esdf/occ; do
  grep -Fq "from=\"$native_visualization\"" \
    "$PROJECT_ROOT/planning/ros_pkgs/sim2real_super_adapter/launch/super_backend.launch" ||
    fail "SUPER visualization topic is not explicitly remapped: $native_visualization"
done
python3 - \
  "$PROJECT_ROOT/planning/ros_pkgs/sim2real_super_adapter/config/planner.yaml" <<'PY'
import sys
import yaml

with open(sys.argv[1], encoding="utf-8") as stream:
    config = yaml.safe_load(stream)
native_values = [
    value
    for key, value in config["fsm"].items()
    if key.endswith(("_topic", "_service"))
]
native_values.extend(
    config["rog_map"]["ros_callback"][key]
    for key in ("odom_topic", "cloud_topic")
)
prefix = "/planning/backends/super/native/"
invalid = [value for value in native_values if not value.startswith(prefix)]
if invalid:
    raise SystemExit(
        "SUPER native interfaces escape {}: {}".format(prefix, invalid)
    )
PY

grep -q 'NLOPT_DEBIAN_VERSION=2.6.1-8ubuntu2' "$PROJECT_ROOT/simulation/Dockerfile" ||
  fail "simulation image is missing the NLopt C++ development package"
grep -q 'NLOPT_DEBIAN_VERSION=2.6.1-8ubuntu2' "$PROJECT_ROOT/deployment/Dockerfile" ||
  fail "real image is missing the NLopt C++ development package"
for dockerfile in \
  "$PROJECT_ROOT/simulation/Dockerfile" \
  "$PROJECT_ROOT/deployment/Dockerfile"; do
  grep -q 'io.sim2real.planner-workspaces="v2"' "$dockerfile" ||
    fail "$dockerfile does not require planner workspace layout v2"
  for dependency in \
    libdw-dev \
    libyaml-cpp-dev \
    ros-noetic-dynamic-reconfigure \
    ros-noetic-message-filters \
    ros-noetic-visualization-msgs; do
    grep -q "$dependency" "$dockerfile" ||
      fail "$dockerfile is missing SUPER dependency $dependency"
  done
done
grep -q '^[[:space:]]*rsync[[:space:]\\]*$' "$PROJECT_ROOT/deployment/Dockerfile" ||
  fail "real image is missing rsync for the staged SUPER source tree"
grep -q '^COPY third_party/SUPER ' "$PROJECT_ROOT/deployment/Dockerfile" ||
  fail "real image does not include the pinned SUPER snapshot"
grep -Fq 'ARG PLANNER_WORKSPACES=interfaces,control,diff' \
  "$PROJECT_ROOT/deployment/Dockerfile" ||
  fail "real image must default to the Diff-only workspace set"
grep -Fq -- '--workspaces interfaces,control,diff' \
  "$PROJECT_ROOT/deployment/Dockerfile" ||
  fail "real image does not explicitly build the Diff-only core"
grep -Fq -- '--workspaces fast,super' \
  "$PROJECT_ROOT/deployment/Dockerfile" ||
  fail "real image does not retain an explicit all-planner opt-in path"
grep -Fq 'PLANNER_BUILD_SET="${REAL_PLANNER_SET:-diff}"' \
  "$PROJECT_ROOT/launch/real_container.sh" ||
  fail "real container launcher must default to the Diff-only image set"
grep -Fq 'io.sim2real.enabled-planner-workspaces' \
  "$PROJECT_ROOT/launch/real_container.sh" ||
  fail "real container launcher does not verify the enabled workspace set"
grep -Fxq 'planning/workspaces' "$PROJECT_ROOT/.dockerignore" ||
  fail "generated planner workspaces must be excluded from Docker build contexts"
grep -Fq 'SELECTED_WORKSPACES="interfaces,control,diff,fast,super"' \
  "$SCRIPT_DIR/build_planner_workspaces.sh" ||
  fail "workspace builder does not include the isolated SUPER domain"
grep -Fq 'rosmsg md5 quadrotor_msgs/PositionCommand' \
  "$SCRIPT_DIR/build_planner_workspaces.sh" ||
  fail "workspace builder does not verify Diff/SUPER message isolation"
grep -Fq 'rsync -a --delete' "$SCRIPT_DIR/build_planner_workspaces.sh" ||
  fail "workspace builder does not stage the pinned SUPER snapshot with rsync"
for container_script in \
  "$PROJECT_ROOT/launch/sim_container.sh" \
  "$PROJECT_ROOT/launch/real_container.sh"; do
  grep -Fq '"v2"' "$container_script" ||
    fail "$container_script does not reject pre-SUPER image layouts"
done
for lifecycle_script in \
  "$PROJECT_ROOT/launch/sim.sh" \
  "$PROJECT_ROOT/launch/real.sh" \
  "$PROJECT_ROOT/launch/sim_container.sh" \
  "$PROJECT_ROOT/launch/real_container.sh" \
  "$PROJECT_ROOT/launch/real_bag.sh"; do
  grep -Fq 'super_backend_adapter_node' "$lifecycle_script" ||
    fail "$lifecycle_script does not recognize the SUPER adapter process"
  grep -Fq 'super_planner/fsm_node' "$lifecycle_script" ||
    fail "$lifecycle_script does not recognize the SUPER native process"
done

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
