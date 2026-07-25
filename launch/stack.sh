#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

usage() {
  cat <<'EOF'
Usage:
  stack.sh sim ACTION [ARGS...]
  stack.sh real ACTION [ARGS...]
  stack.sh sim-container ACTION
  stack.sh real-container ACTION
  stack.sh real-rviz

Examples:
  ./launch/stack.sh sim restart
  ./launch/stack.sh sim goal 2.0 0.0 1.0 0
  ./launch/stack.sh real-container build
  ./launch/stack.sh real restart
  ./launch/stack.sh real goal 1.0 0.0 1.0
EOF
}

target="${1:-}"
if [ -z "$target" ]; then
  usage
  exit 1
fi
shift

case "$target" in
  sim)
    exec "$SCRIPT_DIR/sim.sh" "$@"
    ;;
  real)
    exec "$SCRIPT_DIR/real.sh" "$@"
    ;;
  sim-container)
    exec "$SCRIPT_DIR/sim_container.sh" "$@"
    ;;
  real-container)
    exec "$SCRIPT_DIR/real_container.sh" "$@"
    ;;
  real-rviz)
    exec "$SCRIPT_DIR/real_rviz.sh" "$@"
    ;;
  *)
    usage
    exit 1
    ;;
esac
