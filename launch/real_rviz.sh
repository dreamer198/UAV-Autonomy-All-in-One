#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# The live entrypoint always enables guarded RViz goal forwarding.
# shellcheck disable=SC2034
readonly REAL_RVIZ_ENTRYPOINT_KIND=live
# shellcheck source=launch/real_rviz_common.sh
source "$SCRIPT_DIR/real_rviz_common.sh"
