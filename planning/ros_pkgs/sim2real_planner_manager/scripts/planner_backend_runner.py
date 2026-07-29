#!/usr/bin/env python3
"""roslaunch-friendly entry point for one isolated planner plugin."""

import argparse
import os
from pathlib import Path
import sys

from sim2real_planner_manager.manifest import ManifestError, discover_plugins
from sim2real_planner_manager.runner import run_plugin


def _parse_override(value):
    if "=" not in value:
        raise argparse.ArgumentTypeError("launch override must be NAME=VALUE")
    key, item = value.split("=", 1)
    if not key or not item:
        raise argparse.ArgumentTypeError("launch override must be NAME=VALUE")
    return key, item


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Start one planner plugin from its validated manifest."
    )
    parser.add_argument("--planner", required=True)
    parser.add_argument("--profile", default="")
    parser.add_argument("--manifest-root", default="")
    parser.add_argument("--repository-root", default="")
    parser.add_argument(
        "--runtime-mode", choices=("simulation", "real"), required=True
    )
    parser.add_argument(
        "--arg", action="append", default=[], type=_parse_override
    )
    # roslaunch appends private remapping arguments to executable nodes.
    source_argv = sys.argv[1:] if argv is None else argv
    clean_argv = [
        value
        for value in source_argv
        if not value.startswith("__") and ":=" not in value
    ]
    args = parser.parse_args(clean_argv)
    try:
        manifests = discover_plugins(
            builtin_root=Path(args.manifest_root) if args.manifest_root else None,
            require_runtime=False,
        )
        manifest = manifests[args.planner]
        environment_mode = os.environ.get("SIM2REAL_RUNTIME_MODE", "")
        if environment_mode not in {"simulation", "real"}:
            raise ManifestError(
                "SIM2REAL_RUNTIME_MODE must be explicitly set to simulation or real"
            )
        if environment_mode != args.runtime_mode:
            raise ManifestError(
                "runtime mode {!r} disagrees with container mode {!r}".format(
                    args.runtime_mode, environment_mode
                )
            )
        if not manifest.supports_runtime(args.runtime_mode):
            raise ManifestError(
                "planner {!r} is not enabled for {} runtime".format(
                    manifest.id, args.runtime_mode
                )
            )
        overrides = dict(args.arg)
        return run_plugin(
            manifest,
            repository_root=(
                Path(args.repository_root) if args.repository_root else None
            ),
            profile=args.profile or None,
            overrides=overrides,
            runtime_mode=args.runtime_mode,
        )
    except KeyError:
        parser.error(
            "unknown planner {!r}; discovered: {}".format(
                args.planner, ", ".join(sorted(manifests))
            )
        )
    except ManifestError as exc:
        parser.error(str(exc))
    return 2


if __name__ == "__main__":
    sys.exit(main())
