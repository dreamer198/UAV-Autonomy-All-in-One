#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

export RVIZ_CONFIG_HOST="${RVIZ_CONFIG_HOST:-$PROJECT_ROOT/deployment/config/rviz/offline_bag.rviz}"
export RVIZ_CONFIG_CONTAINER="${RVIZ_CONFIG_CONTAINER:-/root/offline_bag.rviz}"
export START_GOAL_BRIDGE=false

exec "$SCRIPT_DIR/real_rviz.sh"
