#!/usr/bin/env python3
"""Host-side planner manifest discovery and capability gate.

This utility intentionally has no ROS imports, so `sim.sh planners` and
`real.sh planners` work before any catkin workspace has been built.
"""

import argparse
import json
import math
import os
import re
import sys
from pathlib import Path

import yaml


API_VERSION = "sim2real.planner/v1"
ROOT_FIELDS = {
    "api_version",
    "id",
    "ros_namespace",
    "display_name",
    "variant",
    "adapter_node",
    "workspace_setup",
    "launch",
    "default_profile",
    "profiles",
    "timeouts",
    "rates",
    "capabilities",
}
NESTED_FIELDS = {
    "launch": {"package", "file", "arguments"},
    "timeouts": {"startup_sec", "status_sec", "command_sec"},
    "rates": {"status_min_hz", "command_min_hz"},
    "capabilities": {
        "simulation",
        "real_flight",
        "yaw",
        "cancel",
        "goal_validation",
        "rviz",
    },
}
ID_RE = re.compile(r"^[a-z][a-z0-9]*(?:-[a-z0-9]+)*$")
ROS_TOKEN_RE = re.compile(r"^[a-z][a-z0-9_]*$")
ARG_RE = re.compile(r"^[A-Za-z][A-Za-z0-9_]*$")


class ManifestError(RuntimeError):
    pass


def _setup_is_built(path):
    """Recognize catkin linked-devel setups from outside their container.

    catkin_tools creates an absolute setup.bash symlink in linked-devel mode.
    The target is valid at the repository's container mount point but appears
    broken on the host.  The adjacent .catkin marker plus the symlink itself
    are sufficient for the host-side discovery UI; launchers still verify the
    file from inside the selected runtime container.
    """
    return path.is_file() or (
        path.is_symlink() and (path.parent / ".catkin").is_file()
    )


def _expect_mapping(value, label):
    if not isinstance(value, dict):
        raise ManifestError(f"{label} must be a mapping")
    return value


def _expect_exact_fields(mapping, expected, label):
    actual = set(mapping)
    missing = sorted(expected - actual)
    unknown = sorted(actual - expected)
    if missing:
        raise ManifestError(f"{label} is missing fields: {', '.join(missing)}")
    if unknown:
        raise ManifestError(f"{label} has unknown fields: {', '.join(unknown)}")


def _positive_number(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or value <= 0:
        raise ManifestError(f"{label} must be a positive number")


def _candidate_files(manifest_root, extra_path):
    candidates = []

    def add_path(path):
        path = path.expanduser()
        if path.is_file():
            candidates.append(path)
            return
        if not path.is_dir():
            raise ManifestError(f"manifest search path does not exist: {path}")
        direct = path / "planner.plugin.yaml"
        if direct.is_file():
            candidates.append(direct)
        candidates.extend(sorted(path.glob("*/planner.plugin.yaml")))

    add_path(manifest_root)
    if extra_path:
        for raw in extra_path.split(os.pathsep):
            if raw:
                add_path(Path(raw))
    # Resolve after discovery and de-duplicate aliases without changing order.
    unique = []
    seen = set()
    for candidate in candidates:
        resolved = candidate.resolve()
        if resolved not in seen:
            unique.append(resolved)
            seen.add(resolved)
    return unique


def _load_manifest(path, project_root):
    try:
        with path.open("r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream)
    except (OSError, yaml.YAMLError) as exc:
        raise ManifestError(f"{path}: cannot read YAML: {exc}") from exc
    data = _expect_mapping(data, str(path))
    _expect_exact_fields(data, ROOT_FIELDS, str(path))
    if data["api_version"] != API_VERSION:
        raise ManifestError(
            f"{path}: api_version must be {API_VERSION!r}, got {data['api_version']!r}"
        )
    planner_id = data["id"]
    if not isinstance(planner_id, str) or not ID_RE.fullmatch(planner_id):
        raise ManifestError(f"{path}: invalid plugin id {planner_id!r}")
    if path.parent.name != planner_id:
        raise ManifestError(
            f"{path}: directory name {path.parent.name!r} must equal plugin id {planner_id!r}"
        )
    for field in ("display_name", "variant"):
        if not isinstance(data[field], str) or not data[field].strip():
            raise ManifestError(f"{path}: {field} must be a non-empty string")
    adapter_node = data["adapter_node"]
    if (
        not isinstance(adapter_node, str)
        or not ROS_TOKEN_RE.fullmatch(adapter_node)
    ):
        raise ManifestError(f"{path}: invalid adapter_node {adapter_node!r}")
    namespace = data["ros_namespace"]
    if (
        not isinstance(namespace, str)
        or not ROS_TOKEN_RE.fullmatch(namespace)
        or namespace != planner_id.replace("-", "_")
    ):
        raise ManifestError(f"{path}: invalid ros_namespace {namespace!r}")

    setup = data["workspace_setup"]
    if not isinstance(setup, str) or not setup:
        raise ManifestError(f"{path}: workspace_setup must be a non-empty string")
    setup_path = Path(setup)
    if setup_path.is_absolute() or ".." in setup_path.parts:
        raise ManifestError(f"{path}: workspace_setup must be a safe relative path")
    try:
        path.relative_to(project_root / "planning" / "plugins")
        builtin = True
    except ValueError:
        builtin = False
    if setup_path.name != "setup.bash":
        raise ManifestError(f"{path}: workspace_setup must end in setup.bash")
    if builtin and (
        len(setup_path.parts) != 5
        or setup_path.parts[0:2] != ("planning", "workspaces")
        or setup_path.parts[-2:] != ("devel", "setup.bash")
    ):
        raise ManifestError(
            f"{path}: workspace_setup must target planning/workspaces/*_ws/devel/setup.bash"
        )

    for nested_name, fields in NESTED_FIELDS.items():
        nested = _expect_mapping(data[nested_name], f"{path}: {nested_name}")
        _expect_exact_fields(nested, fields, f"{path}: {nested_name}")

    launch = data["launch"]
    for field in ("package", "file"):
        if not isinstance(launch[field], str) or not launch[field]:
            raise ManifestError(f"{path}: launch.{field} must be a non-empty string")
    if not isinstance(launch["arguments"], dict):
        raise ManifestError(f"{path}: launch.arguments must be a mapping")
    for key, value in launch["arguments"].items():
        if (
            not isinstance(key, str)
            or not ARG_RE.fullmatch(key)
            or not isinstance(value, (str, int, float, bool))
            or (
                isinstance(value, (int, float))
                and not isinstance(value, bool)
                and not math.isfinite(float(value))
            )
        ):
            raise ManifestError(
                f"{path}: launch.arguments must contain scalar values keyed by strings"
            )

    profiles = data["profiles"]
    if (
        not isinstance(profiles, list)
        or not profiles
        or any(not isinstance(value, str) or not ROS_TOKEN_RE.fullmatch(value) for value in profiles)
        or len(set(profiles)) != len(profiles)
    ):
        raise ManifestError(f"{path}: profiles must be a non-empty unique string list")
    if data["default_profile"] not in profiles:
        raise ManifestError(f"{path}: default_profile must be included in profiles")

    for field, value in data["timeouts"].items():
        _positive_number(value, f"{path}: timeouts.{field}")
    for field, value in data["rates"].items():
        _positive_number(value, f"{path}: rates.{field}")
    for field, value in data["capabilities"].items():
        if not isinstance(value, bool):
            raise ManifestError(f"{path}: capabilities.{field} must be boolean")

    data["_manifest"] = str(path)
    setup_root = project_root if builtin else path.parent
    setup_candidate = (setup_root / setup_path).absolute()
    data["_setup"] = str(setup_candidate)
    data["_container_setup"] = setup if builtin else str(setup_candidate)
    return data


def discover(project_root, manifest_root, extra_path):
    manifests = {}
    paths = _candidate_files(manifest_root, extra_path)
    if not paths:
        raise ManifestError(f"no planner.plugin.yaml found below {manifest_root}")
    for path in paths:
        manifest = _load_manifest(path, project_root)
        planner_id = manifest["id"]
        if planner_id in manifests:
            raise ManifestError(
                f"duplicate plugin id {planner_id!r}: "
                f"{manifests[planner_id]['_manifest']} and {path}"
            )
        manifests[planner_id] = manifest
    return manifests


def select(manifests, planner_id, mode, profile, require_built):
    try:
        manifest = manifests[planner_id]
    except KeyError as exc:
        raise ManifestError(f"unknown planner id {planner_id!r}") from exc
    capability = "simulation" if mode == "simulation" else "real_flight"
    if not manifest["capabilities"][capability]:
        raise ManifestError(
            f"planner {planner_id!r} is not enabled for {mode.replace('_', ' ')}"
        )
    selected_profile = profile or manifest["default_profile"]
    if selected_profile not in manifest["profiles"]:
        raise ManifestError(
            f"planner {planner_id!r} does not provide profile {selected_profile!r}; "
            f"choose one of: {', '.join(manifest['profiles'])}"
        )
    if require_built and not _setup_is_built(Path(manifest["_setup"])):
        raise ManifestError(
            f"planner {planner_id!r} is not built; missing {manifest['_setup']}"
        )
    result = dict(manifest)
    result["selected_profile"] = selected_profile
    return result


def public_record(manifest):
    return {key: value for key, value in manifest.items() if not key.startswith("_")}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--project-root",
        type=Path,
        default=Path(__file__).resolve().parents[2],
    )
    parser.add_argument("--manifest-root", type=Path)
    subparsers = parser.add_subparsers(dest="command", required=True)

    list_parser = subparsers.add_parser("list")
    list_parser.add_argument("--json", action="store_true")

    resolve_parser = subparsers.add_parser("resolve")
    resolve_parser.add_argument("planner_id")
    resolve_parser.add_argument("--mode", choices=("simulation", "real"), required=True)
    resolve_parser.add_argument("--profile", default="")
    resolve_parser.add_argument("--require-built", action="store_true")
    resolve_parser.add_argument("--json", action="store_true")
    resolve_parser.add_argument(
        "--field",
        choices=(
            "id",
            "profile",
            "workspace_setup",
            "workspace_setup_relative",
            "ros_namespace",
            "launch_package",
            "launch_file",
        ),
    )

    args = parser.parse_args()
    project_root = args.project_root.resolve()
    manifest_root = (
        args.manifest_root.resolve()
        if args.manifest_root
        else project_root / "planning" / "plugins"
    )
    try:
        manifests = discover(
            project_root,
            manifest_root,
            os.environ.get("SIM2REAL_PLANNER_PLUGIN_PATH", ""),
        )
        if args.command == "list":
            records = [manifests[key] for key in sorted(manifests)]
            if args.json:
                print(
                    json.dumps(
                        [public_record(record) for record in records],
                        indent=2,
                        sort_keys=True,
                    )
                )
            else:
                print("ID             DEFAULT    SIM  REAL  BUILT  NAME")
                for manifest in records:
                    setup = Path(manifest["_setup"])
                    print(
                        f"{manifest['id']:<14} "
                        f"{manifest['default_profile']:<10} "
                        f"{'yes' if manifest['capabilities']['simulation'] else 'no':<4} "
                        f"{'yes' if manifest['capabilities']['real_flight'] else 'no':<5} "
                        f"{'yes' if _setup_is_built(setup) else 'no':<6} "
                        f"{manifest['display_name']}"
                    )
            return 0

        manifest = select(
            manifests,
            args.planner_id,
            args.mode,
            args.profile,
            args.require_built,
        )
        if args.field:
            values = {
                "id": manifest["id"],
                "profile": manifest["selected_profile"],
                "workspace_setup": manifest["_setup"],
                "workspace_setup_relative": manifest["_container_setup"],
                "ros_namespace": manifest["ros_namespace"],
                "launch_package": manifest["launch"]["package"],
                "launch_file": manifest["launch"]["file"],
            }
            print(values[args.field])
        else:
            result = public_record(manifest)
            result["selected_profile"] = manifest["selected_profile"]
            result["resolved_workspace_setup"] = manifest["_setup"]
            print(json.dumps(result, indent=2, sort_keys=True))
        return 0
    except ManifestError as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
