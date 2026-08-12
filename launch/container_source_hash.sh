#!/usr/bin/env bash

# Compute a deterministic digest for the files copied into a container image.
# Callers pass repository-relative files/directories that mirror Docker COPY
# sources.  Generated workspaces and language caches are excluded in the same
# way as the repository Docker ignore files.
compute_container_source_hash() {
  if [ "$#" -lt 2 ]; then
    echo "compute_container_source_hash requires a root and at least one source path" >&2
    return 2
  fi

  local source_root="$1"
  shift
  local source_path
  for source_path in "$@"; do
    case "$source_path" in
      /*|..|../*|*/../*|*/..)
        echo "Container hash source must stay below the source root: $source_path" >&2
        return 2
        ;;
    esac
    [ -e "$source_root/$source_path" ] || {
      echo "Container hash source is missing: $source_root/$source_path" >&2
      return 2
    }
  done

  (
    cd "$source_root" || exit 2
    LC_ALL=C tar \
      --sort=name \
      --format=gnu \
      --mtime='@0' \
      --owner=0 \
      --group=0 \
      --numeric-owner \
      --exclude='.git' \
      --exclude='*/.git' \
      --exclude='.git/*' \
      --exclude='*/.git/*' \
      --exclude='__pycache__' \
      --exclude='*/__pycache__' \
      --exclude='*.pyc' \
      --exclude='*.pyo' \
      --exclude='.pytest_cache' \
      --exclude='*/.pytest_cache' \
      --exclude='.mypy_cache' \
      --exclude='*/.mypy_cache' \
      --exclude='.cache' \
      --exclude='*/.cache' \
      --exclude='.catkin_tools' \
      --exclude='*/.catkin_tools' \
      --exclude='build' \
      --exclude='*/build' \
      --exclude='devel' \
      --exclude='*/devel' \
      --exclude='logs' \
      --exclude='*/logs' \
      --exclude='runtime' \
      --exclude='runtime/*' \
      --exclude='planning/workspaces' \
      --exclude='planning/workspaces/*' \
      --exclude='*.bag' \
      -cf - -- "$@" |
      sha256sum |
      awk '{print $1}'
  )
}
