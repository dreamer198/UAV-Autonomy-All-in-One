#!/usr/bin/env python3

import os
import unittest


PROJECT_ROOT = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "..")
)


class SharedRosbagLifecycleTest(unittest.TestCase):
    def read_launcher(self, relative_path):
        path = os.path.join(PROJECT_ROOT, relative_path)
        if not os.path.isfile(path):
            self.skipTest(
                "repository launchers are not mounted into this catkin container"
            )
        with open(path, "r", encoding="utf-8") as stream:
            return stream.read()

    def test_both_stacks_record_and_finalize_split_lz4_bags(self):
        for launcher in ("launch/sim.sh", "launch/real.sh"):
            with self.subTest(launcher=launcher):
                source = self.read_launcher(launcher)
                for expected in (
                    'START_ROSBAG="${',
                    "rosbag record --lz4 --split",
                    "--max-splits='$ROSBAG_MAX_SPLITS'",
                    "--repeat-latched",
                    "--min-space=",
                    'ROSBAG_NODE_NAME="${',
                    "managed_rosbag_pids",
                    "ROSBAG_NODE_REMAP=__name:=",
                    "is_rosbag_record=false",
                    "*/rosbag)",
                    '[ "${argv[$((i + 1))]}" = "record" ]',
                    "__name:='${ROSBAG_NODE_NAME#/}'",
                    "stop_rosbag_gracefully || true",
                    "finalize_indexed_active_bags",
                    "ROSBAG_EXTRA_ARGS_QUOTED",
                    "ROSBAG_TOPICS_QUOTED",
                    "--output-name=*",
                    "--m*",
                ):
                    self.assertIn(expected, source)
                self.assertNotIn('case "${argv[0]}" in', source)
                self.assertNotIn(
                    "pgrep -f '^/opt/ros/noetic/lib/rosbag/record '",
                    source,
                    "shutdown must not target every rosbag recorder in the container",
                )

    def test_container_guards_recognize_noetic_python_rosbag_command(self):
        for launcher in (
            "launch/real_container.sh",
            "launch/sim_container.sh",
            "launch/real_bag.sh",
        ):
            with self.subTest(launcher=launcher):
                source = self.read_launcher(launcher)
                self.assertIn("/rosbag[[:space:]]+record", source)

    def test_both_default_topic_sets_cover_comparable_flight_data(self):
        for launcher in ("launch/sim.sh", "launch/real.sh"):
            with self.subTest(launcher=launcher):
                source = self.read_launcher(launcher)
                for topic in (
                    "/localization/odom",
                    "/localization/cloud_registered",
                    "/mavros/state",
                    "/mavros/setpoint_raw/attitude",
                    "/command/trajectory",
                    "/goal",
                    "/livox/lidar",
                ):
                    self.assertIn(topic, source)

    def test_recording_resource_defaults_match(self):
        sim = self.read_launcher("launch/sim.sh")
        real = self.read_launcher("launch/real.sh")
        for source, prefix in ((sim, "SIM_"), (real, "")):
            with self.subTest(prefix=prefix or "real"):
                for variable, value in (
                    ("START_ROSBAG", "true"),
                    ("ROSBAG_SPLIT_SIZE_MB", "1024"),
                    ("ROSBAG_MAX_SPLITS", "10"),
                    ("ROSBAG_NICE_LEVEL", "10"),
                    ("ROSBAG_MIN_FREE_GB", "5"),
                    ("ROSBAG_STOP_TIMEOUT", "60"),
                    ("ROSBAG_NODE_NAME", "/flight_recorder"),
                ):
                    self.assertIn(
                        '{}="${{{}{}:-{}}}"'.format(
                            variable, prefix, variable, value
                        ),
                        source,
                    )

    def test_simulation_uses_its_persistent_runtime_mount(self):
        source = self.read_launcher("launch/sim.sh")
        self.assertIn(
            'ROSBAG_DIR="${SIM_ROSBAG_DIR:-$DEV_RUNTIME/flight_bags}"',
            source,
        )
        self.assertIn(
            'info "Simulation bags persist on the host under: '
            '$RUNTIME_HOST/flight_bags/"',
            source,
        )

    def test_simulation_rejects_a_detached_runtime_bind_mount(self):
        source = self.read_launcher("launch/sim_container.sh")
        for expected in (
            "verify_live_bind_mount()",
            "stat -Lc '%d:%i'",
            'verify_live_bind_mount "runtime" '
            '"$RUNTIME_HOST" "$RUNTIME_CONTAINER"',
            '"$RUNTIME_HOST/runs"',
            '"$RUNTIME_HOST/active"',
            '"$RUNTIME_HOST/flight_bags"',
            'info "Runtime data: $expected_runtime -> $RUNTIME_CONTAINER"',
        ):
            self.assertIn(expected, source)

    def test_simulation_releases_runtime_mounts_and_keeps_ownership_external(self):
        source = self.read_launcher("launch/sim.sh")
        for expected in (
            'SESSION_MARKER="$LIFECYCLE_LOCK_DIR/'
            'simulation-${SESSION_NAME}.owner"',
            "rosbag_prefix=%s",
            'expected_prefix="$(current_rosbag_output_prefix)"',
            '"$DEV_CONTAINER_SCRIPT" stop',
            "runtime mounts were released",
        ):
            self.assertIn(expected, source)
        self.assertNotIn(
            'SESSION_MARKER="$RUNTIME_HOST/active/',
            source,
        )


if __name__ == "__main__":
    unittest.main()
