#!/usr/bin/env python3
"""Inspect, validate, resolve, or launch planner plugin manifests."""

import argparse
import json
from pathlib import Path
import shlex
import sys

from sim2real_planner_manager.manifest import (
    ManifestError,
    discover_plugins,
)
from sim2real_planner_manager.runner import (
    build_roslaunch_command,
    merged_launch_arguments,
    run_plugin,
)


def _parse_override(value):
    if "=" not in value:
        raise argparse.ArgumentTypeError("launch override must be NAME=VALUE")
    key, item = value.split("=", 1)
    if not key or not item:
        raise argparse.ArgumentTypeError("launch override must be NAME=VALUE")
    return key, item


def _parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest-root", default="")
    parser.add_argument("--repository-root", default="")
    subparsers = parser.add_subparsers(dest="command", required=True)

    list_parser = subparsers.add_parser("list")
    list_parser.add_argument("--json", action="store_true")

    validate_parser = subparsers.add_parser("validate")
    validate_parser.add_argument("ids", nargs="*")
    validate_parser.add_argument("--runtime", action="store_true")
    validate_parser.add_argument("--skip-launch", action="store_true")

    show_parser = subparsers.add_parser("show")
    show_parser.add_argument("id")
    show_parser.add_argument("--json", action="store_true")

    resolve_parser = subparsers.add_parser("resolve")
    resolve_parser.add_argument("id")
    resolve_parser.add_argument("--profile", default="")
    resolve_parser.add_argument("--runtime", action="store_true")
    resolve_parser.add_argument("--arg", action="append", default=[], type=_parse_override)

    launch_parser = subparsers.add_parser("launch")
    launch_parser.add_argument("id")
    launch_parser.add_argument("--profile", default="")
    launch_parser.add_argument(
        "--runtime-mode", choices=("simulation", "real"), required=True
    )
    launch_parser.add_argument("--arg", action="append", default=[], type=_parse_override)
    return parser


def _select(manifests, identifiers, parser):
    if not identifiers:
        return list(manifests.values())
    missing = sorted(set(identifiers) - set(manifests))
    if missing:
        parser.error("unknown planner(s): {}".format(", ".join(missing)))
    return [manifests[item] for item in identifiers]


def main(argv=None):
    parser = _parser()
    args = parser.parse_args(argv)
    root = Path(args.manifest_root) if args.manifest_root else None
    repository_root = (
        Path(args.repository_root) if args.repository_root else None
    )
    try:
        manifests = discover_plugins(builtin_root=root)
        if args.command == "list":
            if args.json:
                print(
                    json.dumps(
                        [manifest.as_dict() for manifest in manifests.values()],
                        sort_keys=True,
                        indent=2,
                    )
                )
            else:
                for manifest in manifests.values():
                    print(
                        "{}\t{}\t{}".format(
                            manifest.id,
                            manifest.variant,
                            ",".join(manifest.profiles),
                        )
                    )
            return 0

        if args.command == "validate":
            selected = _select(manifests, args.ids, parser)
            if args.runtime:
                for manifest in selected:
                    manifest.resolve_runtime(
                        repository_root=repository_root,
                        check_launch=not args.skip_launch,
                    )
            for manifest in selected:
                print("{}: valid".format(manifest.id))
            return 0

        manifest = _select(manifests, [args.id], parser)[0]
        if args.command == "show":
            if args.json:
                print(manifest.to_json())
            else:
                print("id: {}".format(manifest.id))
                print("name: {}".format(manifest.display_name))
                print("variant: {}".format(manifest.variant))
                print("namespace: {}".format(manifest.backend_namespace))
                print("workspace: {}".format(manifest.workspace_setup))
                print(
                    "launch: {} {}".format(
                        manifest.launch.package, manifest.launch.file
                    )
                )
                print("profiles: {}".format(", ".join(manifest.profiles)))
            return 0

        overrides = dict(args.arg)
        if args.command == "resolve":
            arguments = merged_launch_arguments(
                manifest,
                profile=args.profile or None,
                overrides=overrides,
            )
            data = {
                "id": manifest.id,
                "namespace": manifest.backend_namespace,
                "workspace_setup": manifest.workspace_setup,
                "package": manifest.launch.package,
                "launch_file": manifest.launch.file,
                "arguments": arguments,
            }
            if args.runtime:
                command = build_roslaunch_command(
                    manifest,
                    repository_root=repository_root,
                    profile=args.profile or None,
                    overrides=overrides,
                )
                data["command"] = shlex.join(command)
            print(json.dumps(data, sort_keys=True, indent=2))
            return 0

        if args.command == "launch":
            return run_plugin(
                manifest,
                runtime_mode=args.runtime_mode,
                repository_root=repository_root,
                profile=args.profile or None,
                overrides=overrides,
            )
    except ManifestError as exc:
        parser.error(str(exc))
    return 2


if __name__ == "__main__":
    sys.exit(main())
