"""Launch exactly one selected plugin in a clean, isolated environment."""

from __future__ import annotations

import os
from pathlib import Path
import signal
import subprocess
from typing import Any, Dict, Mapping, Optional, Sequence

from .manifest import (
    ManifestError,
    PluginManifest,
    clean_runtime_environment,
)


_RESERVED_ARGUMENTS = frozenset(
    {"backend_id", "backend_namespace", "profile"}
)


def _launch_value(value: Any) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    return str(value)


def merged_launch_arguments(
    manifest: PluginManifest,
    profile: Optional[str] = None,
    overrides: Optional[Mapping[str, Any]] = None,
) -> Dict[str, str]:
    selected_profile = profile or manifest.default_profile
    if not manifest.supports_profile(selected_profile):
        raise ManifestError(
            "plugin {!r} does not support profile {!r}; choose {}".format(
                manifest.id, selected_profile, ", ".join(manifest.profiles)
            )
        )
    merged = {
        key: _launch_value(value)
        for key, value in manifest.launch.arguments.items()
    }
    merged["backend_id"] = manifest.id
    merged["profile"] = selected_profile
    if overrides:
        allowed = set(manifest.launch.arguments) | _RESERVED_ARGUMENTS
        unknown = sorted(set(overrides) - allowed)
        if unknown:
            raise ManifestError(
                "launch overrides are not declared by manifest: {}".format(
                    ", ".join(unknown)
                )
            )
        for key, value in overrides.items():
            merged[key] = _launch_value(value)
    # Plugin identity and selected profile cannot be overridden from a command
    # line. backend_namespace is forced when the launch declares that arg.
    merged["backend_id"] = manifest.id
    merged["profile"] = selected_profile
    if "backend_namespace" in manifest.launch.arguments:
        merged["backend_namespace"] = manifest.ros_namespace
    return merged


def build_roslaunch_command(
    manifest: PluginManifest,
    *,
    repository_root: Optional[Path] = None,
    profile: Optional[str] = None,
    overrides: Optional[Mapping[str, Any]] = None,
) -> Sequence[str]:
    runtime = manifest.resolve_runtime(
        repository_root=repository_root, check_launch=True
    )
    arguments = merged_launch_arguments(
        manifest, profile=profile, overrides=overrides
    )
    roslaunch_arguments = [
        "{}:={}".format(key, value) for key, value in sorted(arguments.items())
    ]
    return [
        "/bin/bash",
        "--noprofile",
        "--norc",
        "-c",
        'set -eo pipefail; set +u; source "$1"; set -u; shift; exec roslaunch "$@"',
        "planner-plugin-launch",
        str(runtime.workspace_setup),
        manifest.launch.package,
        manifest.launch.file,
    ] + roslaunch_arguments


def run_plugin(
    manifest: PluginManifest,
    *,
    runtime_mode: str,
    repository_root: Optional[Path] = None,
    profile: Optional[str] = None,
    overrides: Optional[Mapping[str, Any]] = None,
) -> int:
    environment_mode = os.environ.get("SIM2REAL_RUNTIME_MODE", "")
    if environment_mode not in {"simulation", "real"}:
        raise ManifestError(
            "SIM2REAL_RUNTIME_MODE must be explicitly set to simulation or real"
        )
    if runtime_mode != environment_mode:
        raise ManifestError(
            "runtime mode {!r} disagrees with container mode {!r}".format(
                runtime_mode, environment_mode
            )
        )
    if not manifest.supports_runtime(runtime_mode):
        raise ManifestError(
            "planner {!r} is not enabled for {} runtime".format(
                manifest.id, runtime_mode
            )
        )
    command = build_roslaunch_command(
        manifest,
        repository_root=repository_root,
        profile=profile,
        overrides=overrides,
    )
    child = subprocess.Popen(
        command,
        env=clean_runtime_environment(),
        start_new_session=True,
    )

    previous_handlers = {}

    def forward(signum, _frame):
        if child.poll() is None:
            try:
                os.killpg(child.pid, signum)
            except ProcessLookupError:
                pass

    for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        previous_handlers[signum] = signal.signal(signum, forward)
    try:
        return child.wait()
    finally:
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)
        if child.poll() is None:
            try:
                os.killpg(child.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                child.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(child.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                child.wait()
