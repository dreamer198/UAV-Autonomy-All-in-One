"""Strict parser and discovery support for planner.plugin.yaml files."""

from __future__ import annotations

import json
import math
import os
from dataclasses import asdict, dataclass
from pathlib import Path
import re
import subprocess
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

import yaml


API_VERSION = "sim2real.planner/v1"
MANIFEST_NAME = "planner.plugin.yaml"
PLUGIN_PATH_ENV = "SIM2REAL_PLANNER_PLUGIN_PATH"
_MAX_MANIFEST_BYTES = 256 * 1024
_ID_RE = re.compile(r"^[a-z][a-z0-9]*(?:-[a-z0-9]+)*$")
_ROS_NAME_RE = re.compile(r"^[a-z][a-z0-9_]*$")
_PACKAGE_RE = re.compile(r"^[a-z][a-z0-9_]*$")
_ARG_RE = re.compile(r"^[A-Za-z][A-Za-z0-9_]*$")
_ROOT_KEYS = frozenset(
    {
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
)
_LAUNCH_KEYS = frozenset({"package", "file", "arguments"})
_TIMEOUT_KEYS = frozenset({"startup_sec", "status_sec", "command_sec"})
_RATE_KEYS = frozenset({"status_min_hz", "command_min_hz"})
_CAPABILITY_KEYS = frozenset(
    {"simulation", "yaw", "cancel", "goal_validation", "rviz"}
)


class ManifestError(ValueError):
    """A planner manifest is missing, unsafe, ambiguous, or incompatible."""


class _UniqueKeyLoader(yaml.SafeLoader):
    pass


def _construct_mapping(loader, node, deep=False):
    mapping = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in mapping:
            raise ManifestError("duplicate YAML key: {!r}".format(key))
        mapping[key] = loader.construct_object(value_node, deep=deep)
    return mapping


_UniqueKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _construct_mapping
)


@dataclass(frozen=True)
class LaunchSpec:
    package: str
    file: str
    arguments: Mapping[str, Any]


@dataclass(frozen=True)
class TimeoutSpec:
    startup_sec: float
    status_sec: float
    command_sec: float


@dataclass(frozen=True)
class RateSpec:
    status_min_hz: float
    command_min_hz: float


@dataclass(frozen=True)
class CapabilitySpec:
    simulation: bool
    yaw: bool
    cancel: bool
    goal_validation: bool
    rviz: bool


@dataclass(frozen=True)
class RuntimePaths:
    workspace_setup: Path
    package_path: Optional[Path] = None
    launch_file: Optional[Path] = None


@dataclass(frozen=True)
class PluginManifest:
    api_version: str
    id: str
    ros_namespace: str
    display_name: str
    variant: str
    adapter_node: str
    workspace_setup: str
    launch: LaunchSpec
    default_profile: str
    profiles: Tuple[str, ...]
    timeouts: TimeoutSpec
    rates: RateSpec
    capabilities: CapabilitySpec
    source: Path

    @property
    def backend_namespace(self) -> str:
        return "/planning/backends/{}".format(self.ros_namespace)

    def supports_profile(self, profile: str) -> bool:
        return profile in self.profiles

    def supports_runtime(self, runtime_mode: str) -> bool:
        if runtime_mode == "simulation":
            return self.capabilities.simulation
        if runtime_mode == "real":
            return True
        raise ManifestError(
            "runtime mode must be 'simulation' or 'real', got {!r}".format(
                runtime_mode
            )
        )

    def as_dict(self) -> Dict[str, Any]:
        data = asdict(self)
        data["source"] = str(self.source)
        return data

    def to_json(self) -> str:
        return json.dumps(self.as_dict(), sort_keys=True, indent=2)

    def resolve_runtime(
        self,
        repository_root: Optional[Path] = None,
        check_launch: bool = True,
        timeout_sec: float = 10.0,
    ) -> RuntimePaths:
        if repository_root is not None:
            central_root = Path(repository_root).resolve()
            builtin_root = central_root / "planning" / "plugins"
            try:
                self.source.resolve().relative_to(builtin_root)
                root = central_root
            except ValueError:
                # An environment-discovered plugin is a self-contained,
                # read-only bundle. Its workspace path is relative to the
                # directory containing its manifest.
                root = self.source.resolve().parent
        else:
            try:
                root = _find_repository_root(self.source)
            except ManifestError:
                root = self.source.resolve().parent
        root = root.resolve()
        # Preserve the top-level catkin setup symlink. Resolving that symlink
        # to catkin_tools_prebuild/setup.bash drops this workspace's packages.
        setup = (root / self.workspace_setup).absolute()
        canonical_setup = setup.resolve()
        try:
            canonical_setup.relative_to(root)
        except ValueError as exc:
            raise ManifestError(
                "{}: workspace_setup escapes repository root".format(self.source)
            ) from exc
        if not setup.is_file():
            raise ManifestError(
                "{}: workspace setup does not exist: {}".format(self.id, setup)
            )
        if not os.access(str(setup), os.R_OK):
            raise ManifestError(
                "{}: workspace setup is not readable: {}".format(self.id, setup)
            )
        if not check_launch:
            return RuntimePaths(workspace_setup=setup)

        package_path = _rospack_find(
            setup, self.launch.package, timeout_sec=timeout_sec
        )
        launch_file = (package_path / "launch" / self.launch.file).resolve()
        try:
            launch_file.relative_to(package_path)
        except ValueError as exc:
            raise ManifestError(
                "{}: launch file escapes ROS package".format(self.id)
            ) from exc
        if not launch_file.is_file():
            raise ManifestError(
                "{}: launch file does not exist: {}".format(self.id, launch_file)
            )
        return RuntimePaths(
            workspace_setup=setup,
            package_path=package_path,
            launch_file=launch_file,
        )


def _find_repository_root(source: Path) -> Path:
    start = source.resolve().parent
    for parent in (start,) + tuple(start.parents):
        if (parent / ".git").exists():
            return parent
    # The canonical source layout remains usable after copying without .git.
    for parent in (start,) + tuple(start.parents):
        if (parent / "planning" / "plugins").is_dir():
            return parent
    raise ManifestError(
        "{}: cannot locate repository root; pass --repository-root".format(source)
    )


def clean_runtime_environment() -> Dict[str, str]:
    allowed = {
        "CUDA_VISIBLE_DEVICES",
        "DISPLAY",
        "HOME",
        "LANG",
        "LC_ALL",
        "LOGNAME",
        "NVIDIA_DRIVER_CAPABILITIES",
        "NVIDIA_VISIBLE_DEVICES",
        "PATH",
        "ROS_HOME",
        "ROS_HOSTNAME",
        "ROS_IP",
        "ROS_LOG_DIR",
        "ROS_MASTER_URI",
        "ROS_NAMESPACE",
        "SIM2REAL_RUNTIME_MODE",
        "SIM2REAL_DIFF_PLANNER_CONFIG",
        "USER",
        "XAUTHORITY",
    }
    return {key: value for key, value in os.environ.items() if key in allowed}


def _rospack_find(
    setup: Path, package: str, timeout_sec: float = 10.0
) -> Path:
    command = [
        "/bin/bash",
        "--noprofile",
        "--norc",
        "-c",
        'set -eo pipefail; set +u; source "$1"; set -u; rospack find "$2"',
        "planner-runtime-check",
        str(setup),
        package,
    ]
    try:
        result = subprocess.run(
            command,
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=timeout_sec,
            env=clean_runtime_environment(),
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise ManifestError(
            "cannot inspect ROS package {!r}: {}".format(package, exc)
        ) from exc
    if result.returncode != 0:
        detail = result.stderr.strip() or "rospack returned {}".format(
            result.returncode
        )
        raise ManifestError(
            "cannot resolve ROS package {!r}: {}".format(package, detail)
        )
    lines = [line.strip() for line in result.stdout.splitlines() if line.strip()]
    if len(lines) != 1:
        raise ManifestError(
            "rospack returned an ambiguous path for {!r}".format(package)
        )
    path = Path(lines[0]).resolve()
    if not path.is_dir():
        raise ManifestError(
            "rospack path for {!r} is not a directory: {}".format(package, path)
        )
    return path


def _mapping(value: Any, field: str) -> Mapping[str, Any]:
    if not isinstance(value, dict):
        raise ManifestError("{} must be a mapping".format(field))
    if not all(isinstance(key, str) for key in value):
        raise ManifestError("{} keys must be strings".format(field))
    return value


def _reject_unknown(
    mapping: Mapping[str, Any], expected: Iterable[str], field: str
) -> None:
    expected_set = frozenset(expected)
    unknown = sorted(set(mapping) - expected_set)
    missing = sorted(expected_set - set(mapping))
    if unknown:
        raise ManifestError(
            "{} contains unknown keys: {}".format(field, ", ".join(unknown))
        )
    if missing:
        raise ManifestError(
            "{} is missing keys: {}".format(field, ", ".join(missing))
        )


def _string(value: Any, field: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ManifestError("{} must be a non-empty string".format(field))
    if value != value.strip() or "\x00" in value:
        raise ManifestError("{} contains invalid whitespace or NUL".format(field))
    return value


def _positive_number(value: Any, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ManifestError("{} must be a positive number".format(field))
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise ManifestError("{} must be a finite positive number".format(field))
    return result


def _boolean(value: Any, field: str) -> bool:
    if type(value) is not bool:
        raise ManifestError("{} must be a boolean".format(field))
    return value


def _launch_argument(value: Any, field: str) -> Any:
    if isinstance(value, bool) or isinstance(value, str):
        if isinstance(value, str) and ("\x00" in value or "\n" in value):
            raise ManifestError("{} contains invalid characters".format(field))
        return value
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        if not math.isfinite(float(value)):
            raise ManifestError("{} must be finite".format(field))
        return value
    raise ManifestError(
        "{} must be a string, boolean, or finite number".format(field)
    )


def load_manifest(path: Path) -> PluginManifest:
    path = Path(path)
    try:
        size = path.stat().st_size
    except OSError as exc:
        raise ManifestError("cannot stat manifest {}: {}".format(path, exc)) from exc
    if size <= 0 or size > _MAX_MANIFEST_BYTES:
        raise ManifestError(
            "{} has invalid size {} bytes".format(path, size)
        )
    try:
        text = path.read_text(encoding="utf-8")
    except (OSError, UnicodeError) as exc:
        raise ManifestError("cannot read manifest {}: {}".format(path, exc)) from exc
    try:
        raw = yaml.load(text, Loader=_UniqueKeyLoader)
    except ManifestError:
        raise
    except yaml.YAMLError as exc:
        raise ManifestError("invalid YAML in {}: {}".format(path, exc)) from exc

    root = _mapping(raw, "manifest")
    _reject_unknown(root, _ROOT_KEYS, "manifest")

    api_version = _string(root["api_version"], "api_version")
    if api_version != API_VERSION:
        raise ManifestError(
            "api_version must be {!r}, got {!r}".format(API_VERSION, api_version)
        )
    plugin_id = _string(root["id"], "id")
    if not _ID_RE.fullmatch(plugin_id):
        raise ManifestError("id must match {}".format(_ID_RE.pattern))
    namespace = _string(root["ros_namespace"], "ros_namespace")
    if not _ROS_NAME_RE.fullmatch(namespace):
        raise ManifestError("ros_namespace is not a valid ROS graph token")
    expected_namespace = plugin_id.replace("-", "_")
    if namespace != expected_namespace:
        raise ManifestError(
            "ros_namespace must be {!r} for plugin {!r}".format(
                expected_namespace, plugin_id
            )
        )
    display_name = _string(root["display_name"], "display_name")
    variant = _string(root["variant"], "variant")
    adapter_node = _string(root["adapter_node"], "adapter_node")
    if not _ROS_NAME_RE.fullmatch(adapter_node):
        raise ManifestError("adapter_node is not a valid ROS graph token")

    workspace_setup = _string(root["workspace_setup"], "workspace_setup")
    setup_path = Path(workspace_setup)
    if (
        setup_path.is_absolute()
        or ".." in setup_path.parts
        or setup_path.name != "setup.bash"
    ):
        raise ManifestError(
            "workspace_setup must be a repository-relative setup.bash path"
        )

    launch_raw = _mapping(root["launch"], "launch")
    _reject_unknown(launch_raw, _LAUNCH_KEYS, "launch")
    package = _string(launch_raw["package"], "launch.package")
    if not _PACKAGE_RE.fullmatch(package):
        raise ManifestError("launch.package is not a valid ROS package name")
    launch_file = _string(launch_raw["file"], "launch.file")
    if (
        Path(launch_file).name != launch_file
        or not launch_file.endswith(".launch")
    ):
        raise ManifestError("launch.file must be a .launch basename")
    arguments_raw = _mapping(launch_raw["arguments"], "launch.arguments")
    arguments = {}
    for key, value in arguments_raw.items():
        if not _ARG_RE.fullmatch(key):
            raise ManifestError(
                "launch argument {!r} is not a valid name".format(key)
            )
        arguments[key] = _launch_argument(
            value, "launch.arguments.{}".format(key)
        )

    profiles_value = root["profiles"]
    if not isinstance(profiles_value, list) or not profiles_value:
        raise ManifestError("profiles must be a non-empty list")
    profiles: List[str] = []
    for index, value in enumerate(profiles_value):
        profile = _string(value, "profiles[{}]".format(index))
        if not _ROS_NAME_RE.fullmatch(profile):
            raise ManifestError(
                "profiles[{}] is not a safe launch value".format(index)
            )
        if profile in profiles:
            raise ManifestError("profiles contains duplicate {!r}".format(profile))
        profiles.append(profile)
    default_profile = _string(root["default_profile"], "default_profile")
    if default_profile not in profiles:
        raise ManifestError("default_profile must be listed in profiles")

    timeout_raw = _mapping(root["timeouts"], "timeouts")
    _reject_unknown(timeout_raw, _TIMEOUT_KEYS, "timeouts")
    timeouts = TimeoutSpec(
        startup_sec=_positive_number(
            timeout_raw["startup_sec"], "timeouts.startup_sec"
        ),
        status_sec=_positive_number(
            timeout_raw["status_sec"], "timeouts.status_sec"
        ),
        command_sec=_positive_number(
            timeout_raw["command_sec"], "timeouts.command_sec"
        ),
    )

    rate_raw = _mapping(root["rates"], "rates")
    _reject_unknown(rate_raw, _RATE_KEYS, "rates")
    rates = RateSpec(
        status_min_hz=_positive_number(
            rate_raw["status_min_hz"], "rates.status_min_hz"
        ),
        command_min_hz=_positive_number(
            rate_raw["command_min_hz"], "rates.command_min_hz"
        ),
    )

    capability_raw = _mapping(root["capabilities"], "capabilities")
    _reject_unknown(capability_raw, _CAPABILITY_KEYS, "capabilities")
    capabilities = CapabilitySpec(
        simulation=_boolean(
            capability_raw["simulation"], "capabilities.simulation"
        ),
        yaw=_boolean(capability_raw["yaw"], "capabilities.yaw"),
        cancel=_boolean(capability_raw["cancel"], "capabilities.cancel"),
        goal_validation=_boolean(
            capability_raw["goal_validation"], "capabilities.goal_validation"
        ),
        rviz=_boolean(capability_raw["rviz"], "capabilities.rviz"),
    )

    return PluginManifest(
        api_version=api_version,
        id=plugin_id,
        ros_namespace=namespace,
        display_name=display_name,
        variant=variant,
        adapter_node=adapter_node,
        workspace_setup=workspace_setup,
        launch=LaunchSpec(
            package=package, file=launch_file, arguments=arguments
        ),
        default_profile=default_profile,
        profiles=tuple(profiles),
        timeouts=timeouts,
        rates=rates,
        capabilities=capabilities,
        source=path.resolve(),
    )


def default_builtin_root() -> Optional[Path]:
    here = Path(__file__).resolve()
    for parent in here.parents:
        candidate = parent / "plugins"
        if candidate.is_dir() and any(candidate.glob("*/{}".format(MANIFEST_NAME))):
            return candidate
    return None


def _manifest_paths(root: Path) -> Sequence[Path]:
    root = root.expanduser()
    if root.is_file():
        if root.name != MANIFEST_NAME:
            raise ManifestError(
                "plugin path file must be named {}".format(MANIFEST_NAME)
            )
        return (root.resolve(),)
    if not root.is_dir():
        raise ManifestError("plugin path does not exist: {}".format(root))
    direct = root / MANIFEST_NAME
    if direct.is_file():
        return (direct.resolve(),)
    return tuple(sorted(path.resolve() for path in root.glob("*/" + MANIFEST_NAME)))


def discover_plugins(
    builtin_root: Optional[Path] = None,
    plugin_path: Optional[str] = None,
    require_runtime: bool = False,
    repository_root: Optional[Path] = None,
    check_launch: bool = True,
) -> Dict[str, PluginManifest]:
    roots: List[Path] = []
    if builtin_root is None:
        builtin_root = default_builtin_root()
    if builtin_root is not None:
        roots.append(Path(builtin_root))
    env_value = (
        os.environ.get(PLUGIN_PATH_ENV, "")
        if plugin_path is None
        else plugin_path
    )
    for entry in env_value.split(os.pathsep):
        if entry:
            roots.append(Path(entry))

    paths: List[Path] = []
    seen_paths = set()
    for root in roots:
        for path in _manifest_paths(root):
            if path not in seen_paths:
                seen_paths.add(path)
                paths.append(path)

    manifests: Dict[str, PluginManifest] = {}
    for path in paths:
        manifest = load_manifest(path)
        existing = manifests.get(manifest.id)
        if existing is not None:
            raise ManifestError(
                "duplicate plugin id {!r}: {} and {}".format(
                    manifest.id, existing.source, manifest.source
                )
            )
        if require_runtime:
            manifest.resolve_runtime(
                repository_root=repository_root, check_launch=check_launch
            )
        manifests[manifest.id] = manifest
    return dict(sorted(manifests.items()))
