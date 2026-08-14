#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

PROJECT_ROOT="${SIM2REAL_PROJECT_ROOT:-$DEFAULT_PROJECT_ROOT}"
WORKSPACE_ROOT=""
BASE_UNDERLAY="${SIM2REAL_BASE_UNDERLAY:-/opt/ros/noetic}"
FLAVOR="${SIM2REAL_PLATFORM_FLAVOR:-none}"
BUILD_JOBS="${SIM2REAL_BUILD_JOBS:-4}"
RUN_TESTS=false
SELECTED_WORKSPACES="interfaces,control,diff,fast,super"

usage() {
  cat <<'EOF'
Usage: build_planner_workspaces.sh [OPTIONS]

Build selected isolated ROS planner workspaces. Generated products live below
planning/workspaces and are intentionally not source files. The generic
builder keeps the complete set as its default; deployment images can pass a
smaller explicit set such as interfaces,control,diff.

Options:
  --project-root PATH       Repository root (default: inferred from script)
  --workspace-root PATH     Generated workspace root
  --underlay PATH           Base setup prefix or setup.bash
  --flavor NAME             simulation, deployment, or none
  --jobs N                  Parallel catkin jobs
  --workspaces CSV          Subset of interfaces,control,diff,fast,super
  --test                    Run catkin tests after each selected build
  -h, --help                Show this help
EOF
}

die() {
  echo "[ERROR] $*" >&2
  exit 1
}

info() {
  echo "[INFO] $*"
}

canonical_existing_dir() {
  local path="$1"
  [ -d "$path" ] || die "Directory does not exist: $path"
  realpath -e "$path"
}

resolve_setup() {
  local candidate="$1"
  local setup_path=""
  if [ -f "$candidate" ]; then
    setup_path="$candidate"
  elif [ -f "$candidate/setup.bash" ]; then
    setup_path="$candidate/setup.bash"
  elif [ -f "$candidate/devel/setup.bash" ]; then
    setup_path="$candidate/devel/setup.bash"
  else
    die "No setup.bash found for underlay: $candidate"
  fi
  # Preserve the top-level catkin setup symlink. Resolving it to
  # .private/catkin_tools_prebuild/setup.bash hides the other packages in a
  # linked devel space.
  printf '%s/%s\n' \
    "$(cd "$(dirname "$setup_path")" && pwd -P)" \
    "$(basename "$setup_path")"
}

selected() {
  local wanted="$1"
  case ",$SELECTED_WORKSPACES," in
    *",$wanted,"*) return 0 ;;
    *) return 1 ;;
  esac
}

ensure_source_link() {
  local workspace="$1" name="$2" source_path="$3"
  local destination="$workspace/src/$name"
  [ -e "$source_path" ] || die "Required source is missing: $source_path"
  mkdir -p "$workspace/src"

  if [ -L "$destination" ]; then
    if [ "$(realpath -m "$destination")" = "$(realpath -e "$source_path")" ]; then
      return
    fi
    unlink "$destination"
  elif [ -e "$destination" ]; then
    die "Refusing to replace non-symlink workspace entry: $destination"
  fi
  ln -s "$(realpath -e "$source_path")" "$destination"
}

remove_managed_source_link() {
  local workspace="$1" name="$2"
  local destination="$workspace/src/$name"
  if [ -L "$destination" ]; then
    unlink "$destination"
  elif [ -e "$destination" ]; then
    die "Refusing to remove non-symlink workspace entry: $destination"
  fi
}

sync_managed_source_tree() {
  local workspace="$1" name="$2" source_path="$3"
  local destination="$workspace/src/$name"
  local marker="$destination/.sim2real-managed-source"

  [ -d "$source_path" ] || die "Required source tree is missing: $source_path"
  command -v rsync >/dev/null 2>&1 || die "rsync is required to stage $name"
  mkdir -p "$workspace/src"

  if [ -L "$destination" ]; then
    unlink "$destination"
  elif [ -e "$destination" ] && [ ! -d "$destination" ]; then
    die "Refusing to replace non-directory workspace entry: $destination"
  fi
  if [ -d "$destination" ] && [ ! -f "$marker" ] &&
     [ -n "$(find "$destination" -mindepth 1 -maxdepth 1 -print -quit)" ]; then
    die "Refusing to overwrite unmanaged workspace source tree: $destination"
  fi

  mkdir -p "$destination"
  touch "$marker"
  rsync -a --delete \
    --exclude /.sim2real-managed-source \
    -- "$source_path/" "$destination/"
  touch "$marker"
}

reset_relocated_build_cache() {
  local workspace="$1"
  local metadata="$workspace/build/.catkin_tools.yaml"
  local profile="$workspace/.catkin_tools/profiles/default/config.yaml"
  local configured_workspace=""
  local configured_extend=""
  local generated=""
  local reset_reason=""

  if [ -f "$metadata" ]; then
    configured_workspace="$(
      sed -n 's/^workspace:[[:space:]]*//p' "$metadata" | head -n 1
    )"
    if [ -n "$configured_workspace" ] &&
       [ "$configured_workspace" != "$workspace" ]; then
      reset_reason="$configured_workspace -> $workspace"
    fi
  fi
  if [ -z "$reset_reason" ] && [ -f "$profile" ]; then
    configured_extend="$(
      sed -n 's/^extend_path:[[:space:]]*//p' "$profile" | head -n 1
    )"
    if [ -n "$configured_extend" ] &&
       [ "$configured_extend" != "null" ] &&
       [ ! -d "$configured_extend" ]; then
      reset_reason="missing underlay $configured_extend"
    fi
  fi
  [ -n "$reset_reason" ] || return 0
  [[ "$workspace" == "$WORKSPACE_ROOT/"*_ws ]] ||
    die "Refusing to reset a relocated workspace outside $WORKSPACE_ROOT: $workspace"

  info "Resetting generated cache for relocated workspace: $reset_reason"
  for generated in build devel logs .catkin_tools; do
    rm -rf -- "${workspace:?}/${generated:?}"
  done
}

configure_sources() {
  local interfaces_ws="$WORKSPACE_ROOT/interfaces_ws"
  local control_ws="$WORKSPACE_ROOT/control_ws"
  local diff_ws="$WORKSPACE_ROOT/diff_ws"
  local fast_ws="$WORKSPACE_ROOT/fast_ws"
  local super_ws="$WORKSPACE_ROOT/super_ws"
  local ros_pkgs="$PROJECT_ROOT/planning/ros_pkgs"

  case "$FLAVOR" in
    simulation|deployment|none) ;;
    *) die "--flavor must be simulation, deployment, or none (got: $FLAVOR)" ;;
  esac

  if selected interfaces; then
    ensure_source_link "$interfaces_ws" sim2real_planning_msgs \
      "$ros_pkgs/sim2real_planning_msgs"
  fi

  if selected control; then
    ensure_source_link "$control_ws" sim2real_planner_manager \
      "$ros_pkgs/sim2real_planner_manager"
    ensure_source_link "$control_ws" sim2real_common "$PROJECT_ROOT/common"
    ensure_source_link "$control_ws" se3_controller \
      "$PROJECT_ROOT/third_party/Diff-Planner-PX4/src/se3_controller"

    case "$FLAVOR" in
      simulation)
        ensure_source_link "$control_ws" sim2real_simulation \
          "$PROJECT_ROOT/simulation/ros_pkgs/sim2real_simulation"
        ;;
      deployment)
        ensure_source_link "$control_ws" sim2real_deployment \
          "$PROJECT_ROOT/deployment/ros_pkgs/sim2real_deployment"
        ;;
      none) ;;
    esac
  fi

  if selected diff; then
    # Keep unrelated upstream tools/controllers out of the Diff plugin domain.
    # In particular, several Utils packages rely on undeclared build-order
    # dependencies and are not part of the runtime planner bundle.
    remove_managed_source_link "$diff_ws" Diff-Planner-PX4
    ensure_source_link "$diff_ws" plan_env \
      "$PROJECT_ROOT/third_party/Diff-Planner-PX4/src/diff_planner/plan_env"
    ensure_source_link "$diff_ws" path_searching \
      "$PROJECT_ROOT/third_party/Diff-Planner-PX4/src/diff_planner/path_searching"
    ensure_source_link "$diff_ws" traj_utils \
      "$PROJECT_ROOT/third_party/Diff-Planner-PX4/src/diff_planner/traj_utils"
    ensure_source_link "$diff_ws" traj_opt \
      "$PROJECT_ROOT/third_party/Diff-Planner-PX4/src/diff_planner/traj_opt"
    ensure_source_link "$diff_ws" diff_planner \
      "$PROJECT_ROOT/third_party/Diff-Planner-PX4/src/diff_planner/plan_manage"
    ensure_source_link "$diff_ws" quadrotor_msgs \
      "$PROJECT_ROOT/third_party/Diff-Planner-PX4/src/Utils/quadrotor_msgs"
    ensure_source_link "$diff_ws" sim2real_diff_adapter \
      "$ros_pkgs/sim2real_diff_adapter"
  fi

  if selected fast; then
    ensure_source_link "$fast_ws" Fast-Planner \
      "$PROJECT_ROOT/third_party/Fast-Planner"
    ensure_source_link "$fast_ws" sim2real_fast_adapter \
      "$ros_pkgs/sim2real_fast_adapter"
  fi

  if selected super; then
    # SUPER writes logs below its source tree and uses workspace-relative
    # generated-message includes. Stage the pinned, selected snapshot into this
    # generated writable domain instead of mounting the tracked vendor tree.
    sync_managed_source_tree "$super_ws" SUPER \
      "$PROJECT_ROOT/third_party/SUPER"
    ensure_source_link "$super_ws" sim2real_super_adapter \
      "$ros_pkgs/sim2real_super_adapter"
  fi
}

build_workspace() {
  local label="$1" workspace="$2" underlay_setup="$3"
  local -a catkin_jobs=()
  [ -f "$underlay_setup" ] ||
    die "$label underlay setup is missing: $underlay_setup"
  [[ "$BUILD_JOBS" =~ ^[1-9][0-9]*$ ]] ||
    die "--jobs must be a positive integer (got: $BUILD_JOBS)"
  catkin_jobs=(-j"$BUILD_JOBS" -p"$BUILD_JOBS")

  info "Building $label workspace: $workspace"
  (
    reset_relocated_build_cache "$workspace"
    set +u
    # shellcheck disable=SC1090
    source "$underlay_setup"
    set -u
    cd "$workspace"
    catkin init
    catkin config --extend "${underlay_setup%/setup.bash}" \
      --cmake-args -DCMAKE_BUILD_TYPE=Release
    build_output="$(mktemp)"
    trap 'rm -f "$build_output"' EXIT
    set +e
    catkin build --no-status "${catkin_jobs[@]}" 2>&1 | tee "$build_output"
    build_status="${PIPESTATUS[0]}"
    set -e
    if [ "$build_status" -ne 0 ]; then
      if grep -Eq \
        'internal compiler error: (Segmentation fault|Bus error|Aborted)|cc1plus.*(Segmentation fault|Bus error)' \
        "$build_output"; then
        echo "[WARN] GCC crashed while compiling the overlay; retrying once serially (-j1 -p1)." >&2
        catkin build --no-status -j1 -p1
      else
        exit "$build_status"
      fi
    fi
    rm -f "$build_output"
    trap - EXIT
    if [ "$RUN_TESTS" = "true" ]; then
      catkin test --no-status "${catkin_jobs[@]}"
      if command -v catkin_test_results >/dev/null 2>&1; then
        catkin_test_results --verbose "$workspace/build"
      fi
    fi
  )
}

verify_isolation() {
  local interfaces_setup="$WORKSPACE_ROOT/interfaces_ws/devel/setup.bash"
  local diff_setup="$WORKSPACE_ROOT/diff_ws/devel/setup.bash"
  local fast_setup="$WORKSPACE_ROOT/fast_ws/devel/setup.bash"
  local super_setup="$WORKSPACE_ROOT/super_ws/devel/setup.bash"

  [ ! -f "$interfaces_setup" ] || (
    set +u
    # shellcheck disable=SC1090
    source "$interfaces_setup"
    set -u
    rospack find sim2real_planning_msgs >/dev/null
  )

  if [ -f "$diff_setup" ] && [ -f "$fast_setup" ]; then
    local diff_plan_env fast_plan_env
    diff_plan_env="$(realpath -e "$(
      set +u
      # shellcheck disable=SC1090
      source "$diff_setup"
      set -u
      rospack find plan_env
    )")"
    fast_plan_env="$(realpath -e "$(
      set +u
      # shellcheck disable=SC1090
      source "$fast_setup"
      set -u
      rospack find plan_env
    )")"
    [[ "$diff_plan_env" == "$PROJECT_ROOT/third_party/Diff-Planner-PX4/"* ]] ||
      die "Diff workspace resolves plan_env outside Diff-Planner: $diff_plan_env"
    [[ "$fast_plan_env" == "$PROJECT_ROOT/third_party/Fast-Planner/"* ]] ||
      die "Fast workspace resolves plan_env outside Fast-Planner: $fast_plan_env"
    [ "$diff_plan_env" != "$fast_plan_env" ] ||
      die "Diff and Fast workspaces unexpectedly resolve the same plan_env"
  fi

  if [ -f "$diff_setup" ] && [ -f "$super_setup" ]; then
    local diff_quadrotor_msgs super_quadrotor_msgs
    local diff_position_command_md5 super_position_command_md5
    diff_quadrotor_msgs="$(realpath -e "$(
      set +u
      # shellcheck disable=SC1090
      source "$diff_setup"
      set -u
      rospack find quadrotor_msgs
    )")"
    super_quadrotor_msgs="$(realpath -e "$(
      set +u
      # shellcheck disable=SC1090
      source "$super_setup"
      set -u
      rospack find quadrotor_msgs
    )")"
    [[ "$diff_quadrotor_msgs" == "$PROJECT_ROOT/third_party/Diff-Planner-PX4/"* ]] ||
      die "Diff workspace resolves quadrotor_msgs outside Diff-Planner: $diff_quadrotor_msgs"
    [[ "$super_quadrotor_msgs" == "$WORKSPACE_ROOT/super_ws/src/SUPER/mars_uav_sim/mars_quadrotor_msgs" ]] ||
      die "SUPER workspace resolves quadrotor_msgs outside its staged snapshot: $super_quadrotor_msgs"
    [ "$diff_quadrotor_msgs" != "$super_quadrotor_msgs" ] ||
      die "Diff and SUPER unexpectedly resolve the same quadrotor_msgs package"

    diff_position_command_md5="$(
      set +u
      # shellcheck disable=SC1090
      source "$diff_setup"
      set -u
      rosmsg md5 quadrotor_msgs/PositionCommand
    )"
    super_position_command_md5="$(
      set +u
      # shellcheck disable=SC1090
      source "$super_setup"
      set -u
      rosmsg md5 quadrotor_msgs/PositionCommand
    )"
    [ "$diff_position_command_md5" != "$super_position_command_md5" ] ||
      die "Diff and SUPER PositionCommand schemas unexpectedly have the same MD5"
  fi
  info "Planner workspace isolation checks passed."
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    --project-root)
      [ "$#" -ge 2 ] || die "--project-root requires a path"
      PROJECT_ROOT="$2"
      shift 2
      ;;
    --workspace-root)
      [ "$#" -ge 2 ] || die "--workspace-root requires a path"
      WORKSPACE_ROOT="$2"
      shift 2
      ;;
    --underlay)
      [ "$#" -ge 2 ] || die "--underlay requires a path"
      BASE_UNDERLAY="$2"
      shift 2
      ;;
    --flavor)
      [ "$#" -ge 2 ] || die "--flavor requires a value"
      FLAVOR="$2"
      shift 2
      ;;
    --jobs)
      [ "$#" -ge 2 ] || die "--jobs requires a value"
      BUILD_JOBS="$2"
      shift 2
      ;;
    --workspaces)
      [ "$#" -ge 2 ] || die "--workspaces requires a comma-separated list"
      SELECTED_WORKSPACES="$2"
      shift 2
      ;;
    --test)
      RUN_TESTS=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      die "Unknown argument: $1"
      ;;
  esac
done

PROJECT_ROOT="$(canonical_existing_dir "$PROJECT_ROOT")"
WORKSPACE_ROOT="${WORKSPACE_ROOT:-$PROJECT_ROOT/planning/workspaces}"
mkdir -p "$WORKSPACE_ROOT"
WORKSPACE_ROOT="$(canonical_existing_dir "$WORKSPACE_ROOT")"
BASE_SETUP="$(resolve_setup "$BASE_UNDERLAY")"

# The builder is invoked directly by docker exec, so it cannot rely on a
# login shell or /root/.bashrc to expose ROS tools.
set +u
# shellcheck disable=SC1090
source "$BASE_SETUP"
set -u

case ",$SELECTED_WORKSPACES," in
  *,interfaces,*|*,control,*|*,diff,*|*,fast,*|*,super,*) ;;
  *) die "--workspaces did not select a known workspace" ;;
esac
for requested in ${SELECTED_WORKSPACES//,/ }; do
  case "$requested" in
    interfaces|control|diff|fast|super) ;;
    *) die "Unknown workspace in --workspaces: $requested" ;;
  esac
done

command -v catkin >/dev/null 2>&1 || die "catkin_tools is not installed"
command -v rospack >/dev/null 2>&1 || die "rospack is not installed"

configure_sources

INTERFACES_SETUP="$WORKSPACE_ROOT/interfaces_ws/devel/setup.bash"
if selected interfaces; then
  build_workspace interfaces "$WORKSPACE_ROOT/interfaces_ws" "$BASE_SETUP"
fi
[ -f "$INTERFACES_SETUP" ] ||
  die "interfaces workspace is required by all other workspaces: $INTERFACES_SETUP"

if selected control; then
  build_workspace control "$WORKSPACE_ROOT/control_ws" "$INTERFACES_SETUP"
fi
if selected diff; then
  build_workspace diff "$WORKSPACE_ROOT/diff_ws" "$INTERFACES_SETUP"
fi
if selected fast; then
  build_workspace fast "$WORKSPACE_ROOT/fast_ws" "$INTERFACES_SETUP"
fi
if selected super; then
  build_workspace super "$WORKSPACE_ROOT/super_ws" "$INTERFACES_SETUP"
fi

verify_isolation
info "Requested planner workspaces built successfully."
