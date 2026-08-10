#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

export RVIZ_CONFIG_HOST="${RVIZ_CONFIG_HOST:-$PROJECT_ROOT/deployment/config/rviz/offline_bag.rviz}"
export RVIZ_CONFIG_CONTAINER="${RVIZ_CONFIG_CONTAINER:-/root/offline_bag.rviz}"

# Offline playback has no live planner or validation service. This dedicated
# entrypoint selects the display-only implementation directly; real_rviz.sh
# remains an unconditional live goal-forwarding entrypoint.
# shellcheck disable=SC2034
readonly REAL_RVIZ_ENTRYPOINT_KIND=offline_bag
# shellcheck source=launch/real_rviz_common.sh
source "$SCRIPT_DIR/real_rviz_common.sh"
