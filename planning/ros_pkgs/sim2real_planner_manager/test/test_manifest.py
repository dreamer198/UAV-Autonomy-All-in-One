#!/usr/bin/env python3

import os
from pathlib import Path
import tempfile
import unittest
from unittest import mock

from sim2real_planner_manager.manifest import (
    API_VERSION,
    ManifestError,
    RuntimePaths,
    discover_plugins,
    load_manifest,
)
from sim2real_planner_manager.runner import (
    build_roslaunch_command,
    merged_launch_arguments,
)


REPOSITORY_ROOT = Path(__file__).resolve().parents[4]
PLUGIN_ROOT = REPOSITORY_ROOT / "planning" / "plugins"


class ManifestTest(unittest.TestCase):
    def test_builtin_plugins_are_strictly_discovered(self):
        plugins = discover_plugins(builtin_root=PLUGIN_ROOT, plugin_path="")
        self.assertEqual(
            list(plugins), ["diff", "fast-kino", "fast-topo"]
        )
        self.assertEqual(plugins["diff"].api_version, API_VERSION)
        self.assertEqual(plugins["fast-kino"].ros_namespace, "fast_kino")
        self.assertFalse(plugins["fast-topo"].capabilities.real_flight)

    def test_single_fast_profile_and_identity_are_resolved_deterministically(self):
        plugin = discover_plugins(
            builtin_root=PLUGIN_ROOT, plugin_path=""
        )["fast-kino"]
        arguments = merged_launch_arguments(
            plugin,
            profile=None,
            overrides={
                "backend_id": "malicious",
                "backend_namespace": "malicious",
                "profile": "outdoor",
                "cloud_topic": "/test/cloud",
            },
        )
        self.assertEqual(arguments["backend_id"], "fast-kino")
        self.assertEqual(arguments["backend_namespace"], "fast_kino")
        self.assertEqual(plugin.profiles, ("local",))
        self.assertEqual(arguments["profile"], "local")
        self.assertEqual(arguments["cloud_topic"], "/test/cloud")
        with self.assertRaises(ManifestError):
            merged_launch_arguments(plugin, profile="outdoor")
        with self.assertRaises(ManifestError):
            merged_launch_arguments(plugin, overrides={"undeclared": "x"})

    def test_runtime_capability_is_fail_closed(self):
        plugins = discover_plugins(
            builtin_root=PLUGIN_ROOT, plugin_path=""
        )
        self.assertTrue(plugins["diff"].supports_runtime("real"))
        self.assertTrue(plugins["fast-kino"].supports_runtime("simulation"))
        self.assertFalse(plugins["fast-kino"].supports_runtime("real"))
        with self.assertRaises(ManifestError):
            plugins["diff"].supports_runtime("invalid")

    def test_unknown_and_duplicate_yaml_keys_are_rejected(self):
        source = (PLUGIN_ROOT / "diff" / "planner.plugin.yaml").read_text()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "planner.plugin.yaml"
            path.write_text(source + "\nunknown: true\n")
            with self.assertRaisesRegex(ManifestError, "unknown keys"):
                load_manifest(path)
            path.write_text(source + "\nid: second\n")
            with self.assertRaisesRegex(ManifestError, "duplicate YAML key"):
                load_manifest(path)

    def test_invalid_ros_namespace_is_rejected(self):
        source = (PLUGIN_ROOT / "fast-kino" / "planner.plugin.yaml").read_text()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "planner.plugin.yaml"
            path.write_text(
                source.replace(
                    "ros_namespace: fast_kino",
                    "ros_namespace: fast-kino",
                )
            )
            with self.assertRaisesRegex(ManifestError, "ROS graph token"):
                load_manifest(path)

    def test_duplicate_id_from_external_path_is_rejected(self):
        source = (PLUGIN_ROOT / "diff" / "planner.plugin.yaml").read_text()
        with tempfile.TemporaryDirectory() as directory:
            plugin_dir = Path(directory) / "another"
            plugin_dir.mkdir()
            (plugin_dir / "planner.plugin.yaml").write_text(source)
            with self.assertRaisesRegex(ManifestError, "duplicate plugin id"):
                discover_plugins(
                    builtin_root=PLUGIN_ROOT, plugin_path=directory
                )

    def test_environment_plugin_path_is_read_only_input(self):
        with mock.patch.dict(
            os.environ,
            {"SIM2REAL_PLANNER_PLUGIN_PATH": ""},
            clear=False,
        ):
            plugins = discover_plugins(builtin_root=PLUGIN_ROOT)
        self.assertEqual(set(plugins), {"diff", "fast-kino", "fast-topo"})

    def test_runtime_validation_fails_before_workspace_build(self):
        plugin = discover_plugins(
            builtin_root=PLUGIN_ROOT, plugin_path=""
        )["diff"]
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            with self.assertRaisesRegex(ManifestError, "does not exist"):
                plugin.resolve_runtime(
                    repository_root=root, check_launch=False
                )

    def test_clean_runtime_sources_catkin_before_enabling_nounset(self):
        plugin = discover_plugins(
            builtin_root=PLUGIN_ROOT, plugin_path=""
        )["diff"]
        runtime = RuntimePaths(
            workspace_setup=Path("/tmp/test/devel/setup.bash"),
            package_path=Path("/tmp/test/package"),
            launch_file=Path("/tmp/test/package/launch/plugin.launch"),
        )
        with mock.patch.object(
            type(plugin), "resolve_runtime", return_value=runtime
        ):
            command = build_roslaunch_command(
                plugin, repository_root=REPOSITORY_ROOT
            )
        shell = command[4]
        self.assertIn('set +u; source "$1"; set -u', shell)
        self.assertNotIn("set -euo pipefail", shell)

    def test_external_plugin_workspace_is_relative_to_manifest_bundle(self):
        source = (PLUGIN_ROOT / "diff" / "planner.plugin.yaml").read_text()
        source = source.replace("id: diff", "id: external")
        source = source.replace(
            "ros_namespace: diff", "ros_namespace: external"
        )
        source = source.replace(
            "planning/workspaces/diff_ws/devel/setup.bash",
            "workspace/devel/setup.bash",
        )
        with tempfile.TemporaryDirectory() as directory:
            plugin_dir = Path(directory) / "external"
            setup = plugin_dir / "workspace" / "devel" / "setup.bash"
            setup.parent.mkdir(parents=True)
            setup.write_text("# test setup\n")
            manifest_path = plugin_dir / "planner.plugin.yaml"
            manifest_path.write_text(source)
            manifest = load_manifest(manifest_path)
            runtime = manifest.resolve_runtime(
                repository_root=REPOSITORY_ROOT, check_launch=False
            )
            self.assertEqual(runtime.workspace_setup, setup.absolute())


if __name__ == "__main__":
    unittest.main()
