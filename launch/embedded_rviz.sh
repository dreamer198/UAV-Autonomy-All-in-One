#!/usr/bin/env bash
set -euo pipefail

# Launch the live RViz frame without a pseudo-TTY and print RVIZ_XID so the
# swarm-uav-mapping Qt process can adopt the native X11 window.
REAL_RVIZ_ENTRYPOINT_KIND=live
REAL_RVIZ_EMBEDDED=true
export REAL_RVIZ_ENTRYPOINT_KIND REAL_RVIZ_EMBEDDED
# shellcheck source=real_rviz_common.sh
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/real_rviz_common.sh"
