#!/usr/bin/env python3

import os
import shutil
import subprocess
import tempfile
import unittest


PACKAGE_ROOT = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..")
)
PROJECT_ROOT = os.path.abspath(
    os.environ.get(
        "SIM2REAL_PROJECT_ROOT",
        os.path.join(PACKAGE_ROOT, ".."),
    )
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

    def run_real_container_with_fake_docker(self, action, docker_body):
        launcher = os.path.join(PROJECT_ROOT, "launch", "real_container.sh")
        if not os.path.isfile(launcher):
            self.skipTest("repository real-container launcher is not mounted")
        with tempfile.TemporaryDirectory() as temp_dir:
            bin_dir = os.path.join(temp_dir, "bin")
            runtime_dir = os.path.join(temp_dir, "runtime")
            docker_log = os.path.join(temp_dir, "docker.log")
            os.makedirs(bin_dir)
            fake_docker = os.path.join(bin_dir, "docker")
            with open(fake_docker, "w", encoding="utf-8") as stream:
                stream.write(
                    "#!/usr/bin/env bash\n"
                    "set -eu\n"
                    "printf '%s\\n' \"$*\" >> \"$FAKE_DOCKER_LOG\"\n"
                    + docker_body
                )
            os.chmod(fake_docker, 0o755)
            env = os.environ.copy()
            env["PATH"] = bin_dir + os.pathsep + env.get("PATH", "")
            env["FAKE_DOCKER_LOG"] = docker_log
            env["RUNTIME_DIR"] = runtime_dir
            env["CONTAINER_NAME"] = "fake_real_container"
            env["REAL_SESSION_NAME"] = "fake_real_session"
            completed = subprocess.run(
                [launcher, action],
                cwd=PROJECT_ROOT,
                env=env,
                check=False,
                text=True,
                stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
            )
            with open(docker_log, "r", encoding="utf-8") as stream:
                docker_calls = stream.read()
            runtime_children = []
            if os.path.isdir(runtime_dir):
                runtime_children = os.listdir(runtime_dir)
            return completed, docker_calls, runtime_children

    def run_rviz_with_fake_docker(
        self, entrypoint, *, action_ready=True, goal_dependencies=True
    ):
        with tempfile.TemporaryDirectory() as temp_dir:
            bin_dir = os.path.join(temp_dir, "bin")
            os.makedirs(bin_dir)
            docker_log = os.path.join(temp_dir, "docker.log")
            final_rviz_marker = os.path.join(temp_dir, "final_rviz")
            host_python_log = os.path.join(temp_dir, "host_python.log")
            rviz_config = os.path.join(temp_dir, "review.rviz")
            with open(rviz_config, "w", encoding="utf-8") as stream:
                stream.write("Panels: []\n")

            fake_docker = os.path.join(bin_dir, "docker")
            with open(fake_docker, "w", encoding="utf-8") as stream:
                stream.write(
                    """#!/usr/bin/env bash
set -eu
printf '%s\n' "$*" >> "$FAKE_DOCKER_LOG"
all_args="$*"
case "$all_args" in
  *'io.uav-autonomy-aio.ground-station-layout'*) printf 'v2\n' ;;
  *'io.uav-autonomy-aio.ground-station-source-sha256'*) printf '%s\n' "$FAKE_GROUND_SOURCE_HASH" ;;
  *'image inspect'*'{{.Id}}'*) printf 'sha256:ground-station-test-image\n' ;;
  *'container inspect'*'{{.Image}}'*) printf 'sha256:ground-station-test-image\n' ;;
  *'.State.Running'*) printf 'true\n' ;;
  *'.HostConfig.Init'*) printf 'true\n' ;;
  *'.HostConfig.NetworkMode'*) printf 'host\n' ;;
  *'.HostConfig.IpcMode'*) printf 'host\n' ;;
  *'.HostConfig.Privileged'*) printf 'false\n' ;;
  *'.Config.Env'*) printf 'SIM2REAL_RUNTIME_MODE=ground_station\n' ;;
  *'python3 -c'*'mavros_msgs'*)
    [ "$FAKE_GOAL_DEPENDENCIES" = true ] || exit 1
    ;;
  *'/ground_station/interactive_goal/status'*)
    [ "$FAKE_ACTION_READY" = true ] || exit 1
    ;;
  *'exec rviz -d'*) : > "$FAKE_FINAL_RVIZ_MARKER" ;;
esac
"""
                )
            os.chmod(fake_docker, 0o755)

            for name, body in (
                ("ip", "exit 0\n"),
                ("sleep", "exit 0\n"),
                ("xhost", "exit 1\n"),
                (
                    "python3",
                    'printf "called\\n" >> "$FAKE_HOST_PYTHON_LOG"\nexit 0\n',
                ),
            ):
                path = os.path.join(bin_dir, name)
                with open(path, "w", encoding="utf-8") as stream:
                    stream.write("#!/usr/bin/env bash\n" + body)
                os.chmod(path, 0o755)

            env = os.environ.copy()
            env["PATH"] = bin_dir + os.pathsep + env.get("PATH", "")
            env["FAKE_DOCKER_LOG"] = docker_log
            env["FAKE_FINAL_RVIZ_MARKER"] = final_rviz_marker
            env["FAKE_HOST_PYTHON_LOG"] = host_python_log
            env["FAKE_ACTION_READY"] = str(action_ready).lower()
            env["FAKE_GOAL_DEPENDENCIES"] = str(goal_dependencies).lower()
            hash_completed = subprocess.run(
                [
                    "bash",
                    "-c",
                    'source "$1"; compute_container_source_hash "$2" "${@:3}"',
                    "bash",
                    os.path.join(
                        PROJECT_ROOT, "launch", "container_source_hash.sh"
                    ),
                    PROJECT_ROOT,
                    "deployment/ground_station",
                    "deployment/ros_pkgs/sim2real_ground_station",
                    "planning/ros_pkgs/sim2real_planning_msgs",
                ],
                check=True,
                text=True,
                stdout=subprocess.PIPE,
            )
            env["FAKE_GROUND_SOURCE_HASH"] = hash_completed.stdout.strip()
            env["GROUND_STATION_CONTAINER"] = "review_ground_station"
            env["JETSON_IP"] = "192.0.2.10"
            env["LOCAL_IP"] = "192.0.2.20"
            env["DISPLAY"] = ":99"
            env["RVIZ_CONFIG_HOST"] = rviz_config
            env["RVIZ_CONFIG_CONTAINER"] = "/root/review.rviz"
            env["RVIZ_LOCK_PATH"] = os.path.join(temp_dir, "ground_rviz.lock")
            env.pop("CONTAINER_NAME", None)
            completed = subprocess.run(
                [os.path.join(PROJECT_ROOT, "launch", entrypoint)],
                cwd=PROJECT_ROOT,
                env=env,
                check=False,
                text=True,
                stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
            )
            with open(docker_log, "r", encoding="utf-8") as stream:
                docker_calls = stream.read()
            host_python_called = os.path.exists(host_python_log)
            final_rviz_started = os.path.exists(final_rviz_marker)
            return (
                completed,
                docker_calls,
                host_python_called,
                final_rviz_started,
            )

    def test_sim_and_real_explicitly_enforce_single_vehicle_mode(self):
        sim = self.read("launch/sim.sh")
        real = self.read("launch/real.sh")
        self.assertIn('REQUESTED_DRONE_ID="${SIM_DRONE_ID:-0}"', sim)
        self.assertIn('REQUESTED_DRONE_ID="${DRONE_ID:-0}"', real)
        for source in (sim, real):
            self.assertIn("Only one vehicle is supported", source)
            self.assertIn("--drone-id 0", source)
            self.assertIn("planner_gateway.launch", source)
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

    def test_runtime_container_defaults_are_planner_neutral(self):
        simulation_launchers = (
            "launch/sim.sh",
            "launch/sim_container.sh",
            "launch/outdoor_bag_sim.sh",
        )
        real_launchers = (
            "launch/real.sh",
            "launch/real_container.sh",
            "launch/real_bag.sh",
        )
        for relative_path in simulation_launchers:
            with self.subTest(launcher=relative_path):
                source = self.read(relative_path)
                self.assertIn(
                    "SIM_DEV_CONTAINER:-uav_autonomy_sim", source
                )
        for relative_path in real_launchers:
            with self.subTest(launcher=relative_path):
                source = self.read(relative_path)
                self.assertIn("CONTAINER_NAME:-uav_autonomy_real", source)

        self.assertIn(
            "SIM_DEV_IMAGE:-uav_autonomy_sim:noetic",
            self.read("launch/sim_container.sh"),
        )
        self.assertIn(
            "IMAGE_NAME:-uav_autonomy_real:latest",
            self.read("launch/real_container.sh"),
        )
        ground_station = self.read("launch/ground_station_container.sh")
        self.assertIn(
            "GROUND_STATION_CONTAINER:-${CONTAINER_NAME:-uav_autonomy_ground_station}",
            ground_station,
        )
        self.assertIn(
            "GROUND_STATION_IMAGE:-uav_autonomy_ground_station:noetic",
            ground_station,
        )

        sim_source = self.read("launch/sim.sh")
        self.assertIn("select_owned_session_container", sim_source)
        self.assertIn('export SIM_DEV_CONTAINER="$marker_container"', sim_source)
        self.assertIn(
            "belongs to container '$marker_container'", sim_source
        )

    def test_container_probes_cannot_resolve_same_named_images(self):
        launchers = (
            "launch/sim.sh",
            "launch/sim_container.sh",
            "launch/real.sh",
            "launch/real_container.sh",
            "launch/real_bag.sh",
            "launch/ground_station_container.sh",
            "launch/real_rviz_common.sh",
        )
        for relative_path in launchers:
            with self.subTest(launcher=relative_path):
                source = self.read(relative_path)
                self.assertNotIn("docker inspect", source)
                self.assertIn("docker container inspect", source)

    def test_sim_and_real_require_explicit_planner_for_start(self):
        cases = (
            ("launch/sim.sh", "SIM_PLANNER"),
            ("launch/real.sh", "REAL_PLANNER"),
        )
        for relative_launcher, planner_variable in cases:
            with self.subTest(launcher=relative_launcher):
                launcher = os.path.join(PROJECT_ROOT, relative_launcher)
                source = self.read(relative_launcher)
                self.assertIn(
                    'PLANNER_ID="${' + planner_variable + ':-}"', source
                )
                self.assertNotIn(planner_variable + ":-diff", source)
                self.assertIn("require_planner_selection", source)

                test_env = os.environ.copy()
                test_env.pop(planner_variable, None)
                completed = subprocess.run(
                    ["bash", launcher, "start"],
                    cwd=PROJECT_ROOT,
                    env=test_env,
                    check=False,
                    text=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                )
                self.assertNotEqual(completed.returncode, 0)
                self.assertIn("No planner selected", completed.stdout)

    def test_sim_requires_explicit_scene_for_start(self):
        relative_launcher = "launch/sim.sh"
        launcher = os.path.join(PROJECT_ROOT, relative_launcher)
        source = self.read(relative_launcher)
        self.assertIn('SCENE="${SIM_SCENE:-}"', source)
        self.assertNotIn('SCENE="${SIM_SCENE:-default}"', source)
        self.assertIn("require_scene_selection", source)

        test_env = os.environ.copy()
        test_env.pop("SIM_SCENE", None)
        test_env["SIM_PLANNER"] = "diff"
        completed = subprocess.run(
            ["bash", launcher, "start"],
            cwd=PROJECT_ROOT,
            env=test_env,
            check=False,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
        )
        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("No scene selected", completed.stdout)

    def test_sim_passes_the_validated_scene_to_the_selected_plugin(self):
        sim = self.read("launch/sim.sh")
        self.assertIn(
            "runtime_mode:=simulation scene:=%q", sim
        )
        self.assertIn(
            '"$PLANNER_ID" "$PLANNER_PROFILE" "$SCENE"', sim
        )

    def test_sim_mission_uses_the_same_takeoff_altitude_source_as_arm(self):
        sim = self.read("launch/sim.sh")
        self.assertEqual(
            sim.count(
                "--takeoff-altitude-field '$TAKEOFF_ALTITUDE_FIELD'"
            ),
            2,
        )

    def test_selected_plugin_controls_its_declarative_controller_overlay(self):
        sim = self.read("launch/sim.sh")
        real = self.read("launch/real.sh")
        controller_launch = self.read("common/launch/controller.launch")
        for source in (sim, real):
            self.assertIn(
                "--field controller_config_relative", source
            )
            self.assertIn(
                "planner_config:=$PLANNER_CONTROLLER_CONFIG", source
            )
            self.assertNotIn(
                'if [ "$PLANNER_ID" = "super" ]', source
            )
        self.assertIn(
            '<rosparam command="load" file="$(arg planner_config)"/>',
            controller_launch,
        )

    def test_full_planner_config_override_is_plugin_neutral(self):
        sim = self.read("launch/sim.sh")
        real = self.read("launch/real.sh")
        for source in (sim, real):
            self.assertIn("SIM2REAL_PLANNER_CONFIG", source)
            self.assertNotIn("SIM2REAL_DIFF_PLANNER_CONFIG", source)
        self.assertIn('PLANNER_CONFIG="${SIM_PLANNER_CONFIG:-}"', sim)
        self.assertIn('PLANNER_CONFIG="${PLANNER_CONFIG:-}"', real)
        self.assertNotIn('if [ "$PLANNER_ID" = "diff" ]', real)
        self.assertNotIn("START_DIFF_PLANNER", real)
        self.assertNotIn("PLANNER_RESOLUTION", real)
        self.assertNotIn("PLANNER_OBSTACLES_INFLATION", real)

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
            'ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11312}"',
            'MAVROS_TGT_SYSTEM="${MAVROS_TGT_SYSTEM:-2}"',
            'docker_tmux_cmd "roscore -p $master_port"',
            "armed: True",
            "/mavros/state cannot be verified",
            "stop [--force]",
            "restart [--force]",
            "acquire_start_lock",
            "stop_stack true",
            'START_LOCK="$RUNTIME_DIR/tmp/real.lifecycle.lock"',
            "Another real-flight lifecycle or autonomous command",
            "Real-flight state changed while rosbag was shutting down",
        ):
            self.assertIn(expected, source)
        self.assertGreaterEqual(source.count("require_safe_real_stop"), 3)
        for function_name in (
            "arm_vehicle",
            "publish_goal",
            "request_land",
            "run_waypoint_mission",
            "stop_stack",
        ):
            start = source.index("{}()".format(function_name))
            body = source[start : source.find("\n}", start) + 2]
            self.assertIn("acquire_start_lock", body)
            self.assertIn("release_start_lock", body)
        self.assertIn("verify_live_lifecycle_mount", source)
        acquire_start = source[
            source.index("acquire_start_lock()") : source.index(
                "release_start_lock()"
            )
        ]
        self.assertIn("verify_live_lifecycle_mount", acquire_start)
        self.assertIn("stat -Lc '%d:%i'", source)
        self.assertIn("stat -Lc '%d:%i' -- /root/tmp", source)
        self.assertIn(
            "/ground_station/interactive_goal/status", source
        )
        self.assertIn("/ground_station/flight_command/status", source)

        interactive_goal = self.read(
            "common/scripts/interactive_goal_server.py"
        )
        self.assertIn(
            '"/root/tmp/real.lifecycle.lock"', interactive_goal
        )
        self.assertIn("fcntl.LOCK_EX | fcntl.LOCK_NB", interactive_goal)

        real_launcher = self.read("launch/real.sh")
        self.assertIn(
            'GROUND_VIZ_CLOUD_TOPIC="${GROUND_VIZ_CLOUD_TOPIC:-/ground_station/cloud_registered}"',
            real_launcher,
        )
        self.assertIn(
            'GROUND_VIZ_CLOUD_RATE="${GROUND_VIZ_CLOUD_RATE:-1.0}"',
            real_launcher,
        )
        self.assertIn("rosrun topic_tools throttle messages", real_launcher)
        self.assertIn("ground_station_cloud_throttle", real_launcher)
        self.assertIn('MAVROS_TGT_SYSTEM="${MAVROS_TGT_SYSTEM:-2}"', real_launcher)

    def test_embedded_rviz_bypasses_the_window_manager(self):
        source = self.read(
            "deployment/ros_pkgs/sim2real_ground_station/scripts/embedded_rviz.py"
        )
        dockerfile = self.read("deployment/ground_station/Dockerfile")
        launcher = self.read("launch/ground_station_container.sh")
        self.assertIn("Qt.FramelessWindowHint", source)
        self.assertIn("Qt.X11BypassWindowManagerHint", source)
        self.assertIn("RVIZ_XID=", source)
        self.assertLess(
            source.index("reader.readFile(config, args.config)"),
            source.index("RVIZ_XID="),
        )
        self.assertNotIn('QAction("Target"', source)
        self.assertNotIn('"2D Nav Goal": "Set Goal"', source)
        self.assertIn("fonts-wqy-microhei", dockerfile)
        self.assertIn("WenQuanYi Micro Hei", source)
        self.assertIn("app.setFont(font)", source)
        self.assertIn("fc-match", launcher)
        self.assertIn("SCRIPT_DIRECTORY", source)
        self.assertIn("sys.path.insert(0, SCRIPT_DIRECTORY)", source)

        goal_ui = self.read(
            "deployment/ros_pkgs/sim2real_ground_station/scripts/interactive_goal_ui.py"
        )
        self.assertIn('cancel_text = "取消起飞"', goal_ui)
        self.assertIn("self._flight_client.cancel_goal()", goal_ui)
        self.assertIn("progress.setCancelButton(None)", goal_ui)
        progress_body = goal_ui[
            goal_ui.index("def _open_flight_progress") : goal_ui.index(
                "def _cancel_takeoff"
            )
        ]
        self.assertEqual(progress_body.count("progress.setCancelButton(None)"), 1)
        self.assertGreater(
            progress_body.index("progress.setCancelButton(None)"),
            progress_body.index("else:"),
        )

    def test_live_rviz_helpers_are_owned_by_the_embedding_session(self):
        source = self.read("launch/real_rviz_common.sh")
        self.assertIn(
            'stop_ground_rviz_helpers "$RVIZ_PROCESS_TOKEN"', source
        )
        self.assertGreaterEqual(
            source.count('_ground_rviz_session_token:="$helper_token"'), 2
        )
        self.assertGreaterEqual(
            source.count('"$RVIZ_PROCESS_TOKEN"'), 4
        )

    def test_sim_stop_and_status_respect_stack_ownership_and_health(self):
        source = self.read("launch/sim.sh")
        for expected in (
            'local owned_stack=false',
            'if [ "$owned_stack" = "true" ]; then',
            "no simulation owner session or marker exists; leaving it untouched",
            "local healthy=true",
            '[ "$healthy" = "true" ]',
            'START_LOCK="$RUNTIME_HOST/simulation.lifecycle.lock"',
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
        status_branch = source[
            source.index("status_stack()") : source.index("attach_stack()")
        ]
        self.assertIn(
            "/ground_station/interactive_goal/status", status_branch
        )
        self.assertIn("/ground_station/flight_command/status", status_branch)

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
        self.assertIn("live_bind_mount_current", real)
        self.assertIn("stat -Lc '%d:%i'", real)
        self.assertIn(
            'live_bind_mount_current "$RUNTIME_DIR/tmp" /root/tmp', real
        )
        self.assertIn("--init", real)
        self.assertIn(
            '--label "$IMAGE_SOURCE_LABEL=$source_hash"', real
        )
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
        real_start = self.read("launch/real.sh")
        start_body = real_start[
            real_start.index("start_stack()") : real_start.index(
                "stop_stack()"
            )
        ]
        verify_call = '"$SCRIPT_DIR/real_container.sh" verify'
        self.assertIn(verify_call, start_body)
        self.assertLess(
            start_body.index(verify_call),
            start_body.index("ensure_container_running"),
        )

    def test_real_verify_rejects_a_stale_source_label_without_mutating(self):
        completed, docker_calls, runtime_children = (
            self.run_real_container_with_fake_docker(
                "verify",
                """
case "$*" in
  *'io.sim2real.planner-workspaces'*) printf 'v2\n' ;;
  *'io.uav-autonomy-aio.real-source-sha256'*) printf 'stale-source\n' ;;
esac
""",
            )
        )
        self.assertNotEqual(0, completed.returncode)
        self.assertIn("missing or stale", completed.stdout)
        self.assertNotIn("build ", docker_calls)
        self.assertEqual([], runtime_children)

    def test_real_build_is_rejected_before_docker_build_when_stack_active(self):
        completed, docker_calls, _runtime_children = (
            self.run_real_container_with_fake_docker(
                "build",
                """
case "$*" in
  *'container inspect -f {{.State.Running}}'*) printf 'true\n' ;;
  *'top fake_real_container'*) printf 'python3 /root/flight_command_server.py\n' ;;
  *'exec -i fake_real_container'*) exit 1 ;;
esac
""",
            )
        )
        self.assertNotEqual(0, completed.returncode)
        self.assertIn("while the real stack or recorder is active", completed.stdout)
        self.assertNotIn("build ", docker_calls)

    def test_real_deployment_separates_jetson_and_ground_station_containers(self):
        real = self.read("launch/real_container.sh")
        ground = self.read("launch/ground_station_container.sh")
        rviz = self.read("launch/real_rviz_common.sh")
        live_rviz = self.read("launch/real_rviz.sh")
        ground_dockerfile = self.read("deployment/ground_station/Dockerfile")

        self.assertIn("Run it\non the Jetson only", real)
        self.assertIn("--network host", ground)
        self.assertIn("SIM2REAL_RUNTIME_MODE=ground_station", ground)
        self.assertIn("HostConfig.Privileged", ground)
        self.assertIn('= "false"', ground)
        self.assertNotIn("FCU_DEVICE", ground)
        self.assertNotIn("--privileged\n", ground)
        self.assertIn("ros-noetic-rviz", ground_dockerfile)
        self.assertIn("ros-noetic-mavros-msgs", ground_dockerfile)
        self.assertIn("python3-numpy", ground_dockerfile)
        self.assertIn("ros-noetic-tf2-ros", ground_dockerfile)
        self.assertIn("ros-noetic-visualization-msgs", ground_dockerfile)
        self.assertIn("ros-noetic-std-srvs", ground_dockerfile)
        self.assertIn("sim2real_planning_msgs", ground_dockerfile)
        self.assertNotIn("COPY third_party", ground_dockerfile)
        self.assertIn("compute_image_source_hash", ground)
        self.assertIn("ground-station-source-sha256", ground)
        self.assertIn('--label "$IMAGE_SOURCE_LABEL=$source_hash"', ground)
        self.assertIn(
            "GROUND_STATION_CONTAINER:-${CONTAINER_NAME:-uav_autonomy_ground_station}",
            rviz,
        )
        self.assertIn("ground_station_container.sh run", rviz)
        self.assertIn("readonly REAL_RVIZ_ENTRYPOINT_KIND=live", live_rviz)
        self.assertTrue(
            os.access(
                os.path.join(
                    PROJECT_ROOT, "launch", "ground_station_container.sh"
                ),
                os.X_OK,
            )
        )

    def test_planner_builder_sources_underlay_before_ros_tool_checks(self):
        source = self.read("planning/scripts/build_planner_workspaces.sh")
        setup_source = 'source "$BASE_SETUP"'
        rospack_check = (
            'command -v rospack >/dev/null 2>&1 || '
            'die "rospack is not installed"'
        )
        self.assertIn(setup_source, source)
        self.assertIn(rospack_check, source)
        self.assertLess(
            source.index(setup_source, source.index("BASE_SETUP=")),
            source.index(rospack_check),
        )
        self.assertIn("Preserve the top-level catkin setup symlink", source)
        self.assertIn("reset_relocated_build_cache", source)
        self.assertIn(
            '[[ "$workspace" == "$WORKSPACE_ROOT/"*_ws ]]', source
        )

    def test_fast_planner_node_waits_for_generated_message_headers(self):
        source = self.read(
            "third_party/Fast-Planner/fast_planner/plan_manage/CMakeLists.txt"
        )
        target_start = source.index("add_executable(fast_planner_node")
        target_end = source.index("target_link_libraries(fast_planner_node")
        target_definition = source[target_start:target_end]
        self.assertIn("add_dependencies(fast_planner_node", target_definition)
        self.assertIn(
            "${${PROJECT_NAME}_EXPORTED_TARGETS}", target_definition
        )
        self.assertIn("${catkin_EXPORTED_TARGETS}", target_definition)

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
                    "container=uav_autonomy_real\n"
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

    def test_real_rviz_uses_fixed_wired_link_and_guarded_action(self):
        source = self.read("launch/real_rviz_common.sh")
        live = self.read("launch/real_rviz.sh")
        offline = self.read("launch/real_bag_rviz.sh")
        self.assertIn('JETSON_IP="${JETSON_IP:-192.168.1.123}"', source)
        self.assertIn('LOCAL_IP="${LOCAL_IP:-192.168.1.124}"', source)
        self.assertIn('ROS_MASTER_PORT="${ROS_MASTER_PORT:-11312}"', source)
        self.assertIn("action_ready", source)
        self.assertNotIn("visualization_ready", source)
        self.assertIn(
            "/ground_station/interactive_goal/status", source
        )
        self.assertIn("master_has_publishers", source)
        self.assertIn("socket.setdefaulttimeout(2.0)", source)
        self.assertIn("RVIZ_HELPER_START_DELAY", source)
        self.assertIn("sleep \"$RVIZ_HELPER_START_DELAY\"", source)
        self.assertIn("_input_topic:=/ground_station/cloud_registered", source)
        for expected in (
            "stable_environment_viz.py",
            "flight_visualization.py",
            "__name:=ground_rviz_environment",
            "__name:=ground_rviz_flight_visualization",
            "/planning/viz/environment",
            "/planning/viz/active_goal",
            "/planning/viz/executed_path",
        ):
            self.assertIn(expected, source)
        self.assertNotIn("rviz_goal_to_diff_planner", source)
        self.assertIn("ground_rviz.lock", source)
        self.assertIn("trap cleanup EXIT", source)
        self.assertIn("timeout 5 docker exec", source)
        self.assertIn("stop_ground_rviz_helpers", source)
        self.assertIn("/proc/[0-9]*/cmdline", source)
        self.assertNotIn("timeout 3 rosnode kill", source)
        self.assertIn("xhost -SI:localuser:root", source)
        self.assertNotIn("xhost +local:docker", source)
        self.assertNotIn("START_GOAL_BRIDGE", source)
        self.assertNotIn("START_GOAL_BRIDGE", live)
        self.assertNotIn("START_GOAL_BRIDGE", offline)
        self.assertNotIn("_REAL_RVIZ_SESSION_KIND", source)
        self.assertNotIn("_REAL_RVIZ_SESSION_KIND", live)
        self.assertNotIn("_REAL_RVIZ_SESSION_KIND", offline)
        self.assertIn("readonly REAL_RVIZ_ENTRYPOINT_KIND=live", live)
        self.assertIn(
            "readonly REAL_RVIZ_ENTRYPOINT_KIND=offline_bag", offline
        )
        self.assertIn("/ground_station/interactive_goal", source)

        panel = self.read(
            "deployment/ros_pkgs/sim2real_ground_station/src/interactive_goal_panel.cpp"
        )
        self.assertIn("confirmation.setDefaultButton(QMessageBox::Cancel)", panel)
        self.assertIn("LANDED_STATE_ON_GROUND", panel)
        self.assertIn('state.mode != "OFFBOARD"', panel)

    def test_live_rviz_requires_onboard_action_readiness(self):
        completed, docker_calls, _, final_rviz_started = (
            self.run_rviz_with_fake_docker(
                "real_rviz.sh", action_ready=False
            )
        )
        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("ground-station actions are unavailable", completed.stdout)
        self.assertIn(
            "/ground_station/interactive_goal/status", docker_calls
        )
        self.assertIn("/ground_station/flight_command/status", docker_calls)
        self.assertFalse(final_rviz_started)

    def test_live_rviz_starts_after_action_readiness(self):
        completed, docker_calls, host_python_called, final_rviz_started = (
            self.run_rviz_with_fake_docker("real_rviz.sh")
        )
        self.assertEqual(0, completed.returncode, completed.stdout)
        self.assertNotIn("rviz_goal_to_diff_planner.py", docker_calls)
        self.assertIn("/ground_station/interactive_goal/status", docker_calls)
        self.assertIn("/ground_station/flight_command/status", docker_calls)
        self.assertIn("stable_environment_viz.py", docker_calls)
        self.assertIn("flight_visualization.py", docker_calls)
        action_check = next(
            line
            for line in docker_calls.splitlines()
            if "python3 - http://" in line
            and "/ground_station/interactive_goal/status" in line
        )
        helper_start = next(
            line
            for line in docker_calls.splitlines()
            if line.startswith("exec -d")
            and "RVIZ_HELPER_START_DELAY" in line
        )
        self.assertLess(
            docker_calls.index(action_check), docker_calls.index(helper_start)
        )
        self.assertFalse(host_python_called)
        self.assertTrue(final_rviz_started)

    def test_offline_rviz_never_requires_or_mutates_goal_bridge(self):
        completed, docker_calls, host_python_called, final_rviz_started = (
            self.run_rviz_with_fake_docker(
                "real_bag_rviz.sh",
                action_ready=False,
                goal_dependencies=False,
            )
        )
        self.assertEqual(0, completed.returncode, completed.stdout)
        self.assertNotIn("mavros_msgs", docker_calls)
        self.assertNotIn("rviz_goal_to_diff_planner", docker_calls)
        self.assertNotIn("stable_environment_viz.py", docker_calls)
        self.assertNotIn("flight_visualization.py", docker_calls)
        self.assertFalse(host_python_called)
        self.assertTrue(final_rviz_started)

    def test_ground_station_image_installs_mavros_telemetry_helper(self):
        helper = self.read(
            "deployment/ros_pkgs/sim2real_ground_station/scripts/"
            "ground_station_telemetry.py"
        )
        cmake = self.read(
            "deployment/ros_pkgs/sim2real_ground_station/CMakeLists.txt"
        )
        launcher = self.read("launch/ground_station_container.sh")
        dockerfile = self.read("deployment/ground_station/Dockerfile")

        self.assertIn("scripts/ground_station_telemetry.py", cmake)
        for topic in (
            '"/mavros/state"',
            '"/mavros/extended_state"',
            '"/localization/odom"',
            '"/mavros/battery"',
            '"/mavros/global_position/global"',
        ):
            self.assertIn(topic, helper)
        self.assertIn("if state is None:", helper)
        self.assertNotIn("state is None or odometry is None", helper)
        installed_path = (
            "/root/ground_station_ws/devel/lib/"
            "sim2real_ground_station/ground_station_telemetry.py"
        )
        self.assertIn(installed_path, launcher)
        self.assertIn(installed_path, dockerfile)


if __name__ == "__main__":
    unittest.main()
