#!/usr/bin/env python3

import os
import shutil
import subprocess
import tempfile
import unittest


PROJECT_ROOT = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "..")
)


class ShellSafetyContractsTest(unittest.TestCase):
    def read(self, relative_path):
        path = os.path.join(PROJECT_ROOT, relative_path)
        if not os.path.isfile(path):
            self.skipTest(
                "repository launchers are not mounted into this catkin container"
            )
        with open(path, "r", encoding="utf-8") as stream:
            return stream.read()

    def test_sim_and_real_explicitly_enforce_single_vehicle_mode(self):
        sim = self.read("launch/sim.sh")
        real = self.read("launch/real.sh")
        self.assertIn('REQUESTED_DRONE_ID="${SIM_DRONE_ID:-0}"', sim)
        self.assertIn('REQUESTED_DRONE_ID="${DRONE_ID:-0}"', real)
        for source in (sim, real):
            self.assertIn("Only one vehicle is supported", source)
            self.assertIn("--drone-id 0", source)
            self.assertIn("drone_id:=0", source)
        self.assertIn("DRONE_ID=0", real)
        self.assertNotIn("/drone_${DRONE_ID}_planning", sim)
        self.assertNotIn("/drone_${DRONE_ID}_planning", real)

        test_env = os.environ.copy()
        for name in ("SIM_START_PLANNER", "SIM_ROSBAG_EXTRA_ARGS"):
            test_env.pop(name, None)
        test_env["SIM_DRONE_ID"] = "1"
        completed = subprocess.run(
            ["bash", os.path.join(PROJECT_ROOT, "launch", "sim.sh"), "status"],
            cwd=PROJECT_ROOT,
            env=test_env,
            check=False,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
        )
        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("Only one vehicle is supported", completed.stdout)

    def test_sim_rejects_invalid_start_boolean_and_recorder_ownership_override(self):
        launcher = os.path.join(PROJECT_ROOT, "launch", "sim.sh")
        if not os.path.isfile(launcher):
            self.skipTest(
                "repository launchers are not mounted into this catkin container"
            )
        for variable, value, expected in (
            ("SIM_START_PLANNER", "yes", "must be exactly 'true' or 'false'"),
            (
                "SIM_ROSBAG_EXTRA_ARGS",
                "-O /tmp/foreign",
                "may not override managed output",
            ),
            (
                "SIM_ROSBAG_EXTRA_ARGS",
                "--output-name=/tmp/foreign",
                "may not override managed output",
            ),
            (
                "SIM_ROSBAG_EXTRA_ARGS",
                "--max-splits=1000",
                "may not override managed output",
            ),
            (
                "SIM_ROSBAG_EXTRA_ARGS",
                "--output-n=/tmp/foreign",
                "may not override managed output",
            ),
            (
                "SIM_ROSBAG_EXTRA_ARGS",
                "--max-sp=1000",
                "may not override managed output",
            ),
            (
                "SIM_ROSBAG_EXTRA_ARGS",
                "--si=4096",
                "may not override managed output",
            ),
            (
                "SIM_ROSBAG_TOPICS",
                "/mavros/state --output-n=/tmp/foreign",
                "contains an invalid absolute ROS topic",
            ),
        ):
            with self.subTest(variable=variable):
                test_env = os.environ.copy()
                for name in (
                    "SIM_DRONE_ID",
                    "SIM_START_PLANNER",
                    "SIM_ROSBAG_EXTRA_ARGS",
                    "SIM_ROSBAG_TOPICS",
                ):
                    test_env.pop(name, None)
                test_env[variable] = value
                completed = subprocess.run(
                    ["bash", launcher, "status"],
                    cwd=PROJECT_ROOT,
                    env=test_env,
                    check=False,
                    text=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                )
                self.assertNotEqual(completed.returncode, 0)
                self.assertIn(expected, completed.stdout)

    def test_real_shutdown_has_armed_and_unknown_state_interlocks(self):
        source = self.read("launch/real.sh")
        for expected in (
            "require_safe_real_stop",
            "mavros_state_snapshot",
            "armed: True",
            "/mavros/state cannot be verified",
            "stop [--force]",
            "restart [--force]",
            "acquire_start_lock",
            "stop_stack true",
            'START_LOCK="$LIFECYCLE_LOCK_DIR/real.lifecycle.lock"',
            "Another real-flight lifecycle or autonomous command",
            "Real-flight state changed while rosbag was shutting down",
        ):
            self.assertIn(expected, source)
        self.assertGreaterEqual(source.count("require_safe_real_stop"), 3)
        for function_name in (
            "arm_vehicle",
            "publish_goal",
            "run_waypoint_mission",
            "stop_stack",
        ):
            start = source.index("{}()".format(function_name))
            body = source[start : source.find("\n}", start) + 2]
            self.assertIn("acquire_start_lock", body)
            self.assertIn("release_start_lock", body)

    def test_sim_stop_and_status_respect_stack_ownership_and_health(self):
        source = self.read("launch/sim.sh")
        for expected in (
            'local owned_stack=false',
            'if [ "$owned_stack" = "true" ]; then',
            "no simulation owner session or marker exists; leaving it untouched",
            "local healthy=true",
            '[ "$healthy" = "true" ]',
            'START_LOCK="$LIFECYCLE_LOCK_DIR/simulation.lifecycle.lock"',
            "first_simulation_owner_marker",
            "has no repository ownership marker",
        ):
            self.assertIn(expected, source)
        ownership_branch = source[
            source.index("stop_stack()") : source.index("status_stack()")
        ]
        self.assertLess(
            ownership_branch.index('if [ -f "$SESSION_MARKER" ]'),
            ownership_branch.index("elif tmux_has_session"),
        )

    def test_simulated_land_sends_and_checks_setbool_true(self):
        source = self.read("launch/sim.sh")
        self.assertIn(
            'rosservice call /land \\"data: true\\"', source
        )
        self.assertIn("grep -q 'success: True'", source)
        self.assertNotIn("rosservice call /land '{}'", source)

    def test_ros_node_scripts_remain_executable(self):
        script_roots = (
            "common/scripts",
            "deployment/ros_pkgs/sim2real_deployment/scripts",
            "simulation/ros_pkgs/sim2real_simulation/scripts",
        )
        missing = []
        for relative_root in script_roots:
            root = os.path.join(PROJECT_ROOT, relative_root)
            if not os.path.isdir(root):
                self.skipTest(
                    "repository ROS scripts are not mounted into this catkin "
                    "container"
                )
            for name in os.listdir(root):
                path = os.path.join(root, name)
                if name.endswith(".py") and not os.access(path, os.X_OK):
                    missing.append(os.path.relpath(path, PROJECT_ROOT))
        self.assertEqual([], missing)

    def test_container_mutations_have_active_stack_interlocks(self):
        real = self.read("launch/real_container.sh")
        sim = self.read("launch/sim_container.sh")
        self.assertIn("require_inactive_real_stack", real)
        self.assertIn("PX4 reports armed", real)
        self.assertIn("container_layout_current", real)
        self.assertIn("--init", real)
        self.assertIn("require_inactive_simulation", sim)
        self.assertIn("active_simulation_detected", sim)
        for source in (real, sim):
            self.assertIn("--force", source)
            self.assertIn("read -r -a parsed", source)
            self.assertNotIn("local extra_args=(", source)
        restart_body = sim[
            sim.index("    restart)") : sim.index("    recreate)")
        ]
        self.assertIn("require_inactive_simulation", restart_body)

    def test_outdoor_scene_publish_is_bounded_and_read_only_actions_do_not_generate(self):
        source = self.read("launch/outdoor_bag_sim.sh")
        for expected in (
            "validate_publish_paths",
            'rsync -a --delete -- "$OUTPUT_HOST/" "$ASSET_HOST/"',
            "OUTDOOR_SIM_OUTPUT_HOST must stay below",
            "OUTDOOR_SIM_ASSET_HOST must stay below",
        ):
            self.assertIn(expected, source)
        case_body = source[source.index('case "$action" in') :]
        start_body = case_body[
            case_body.index("start)") : case_body.index("shell)")
        ]
        self.assertIn("ensure_scene", start_body)
        passive_body = case_body[case_body.index("stop|status|attach|arm|land|goal)") :]
        self.assertNotIn("ensure_scene", passive_body)

    def test_offline_bag_playback_owns_every_process_it_stops(self):
        source = self.read("launch/real_bag.sh")
        for expected in (
            "OFFLINE_BAG_MASTER_TOKEN",
            "owned_master_pid",
            "__name:=offline_bag_player",
            "__name:=offline_livox_converter",
            "__name:=offline_body_to_livox_tf",
            "fastlio_mapping",
            "drone_0_",
            "math.isfinite(value) and value > 0.0",
            "PLAYBACK_MARKER",
            "PLAYBACK_TOKEN_OPTION",
            "playback_session_owned",
            "not owned by this playback launcher",
        ):
            self.assertIn(expected, source)
        self.assertNotIn('for pattern in "[r]osbag play"', source)

    def test_offline_bag_stop_preserves_another_sessions_markers(self):
        launcher = os.path.join(PROJECT_ROOT, "launch", "real_bag.sh")
        if not os.path.isfile(launcher):
            self.skipTest(
                "repository launchers are not mounted into this catkin "
                "container"
            )
        if not shutil.which("docker") or not shutil.which("tmux"):
            self.skipTest("docker and tmux are required for launcher validation")
        with tempfile.TemporaryDirectory() as runtime_dir:
            playback_marker = os.path.join(
                runtime_dir, ".real_bag_playback_owned"
            )
            master_marker = os.path.join(
                runtime_dir, ".real_bag_roscore_owned"
            )
            with open(playback_marker, "w", encoding="utf-8") as stream:
                stream.write("session=session_a\ntoken=token_a\n")
            with open(master_marker, "w", encoding="utf-8") as stream:
                stream.write(
                    "container=diff_planner_px4_real\n"
                    "session=session_a\n"
                    "token=token_a\n"
                )
            env = os.environ.copy()
            env["RUNTIME_DIR"] = runtime_dir
            env["BAG_SESSION_NAME"] = "session_b"
            completed = subprocess.run(
                [launcher, "stop"],
                cwd=PROJECT_ROOT,
                env=env,
                check=False,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
            )
            self.assertNotEqual(completed.returncode, 0)
            self.assertIn("belongs to another session", completed.stdout)
            self.assertTrue(os.path.isfile(playback_marker))
            self.assertTrue(os.path.isfile(master_marker))

    def test_real_rviz_uses_route_detection_and_cleans_up_bridge_and_xhost(self):
        source = self.read("launch/real_rviz.sh")
        self.assertIn('JETSON_IP="${JETSON_IP:-}"', source)
        self.assertIn('RVIZ_GOAL_Z="${RVIZ_GOAL_Z:-1.0}"', source)
        self.assertIn('ip -4 route get "$JETSON_IP"', source)
        self.assertIn("bridge_ready", source)
        self.assertIn("trap cleanup EXIT", source)
        self.assertIn("xhost -SI:localuser:root", source)
        self.assertNotIn("xhost +local:docker", source)


if __name__ == "__main__":
    unittest.main()
