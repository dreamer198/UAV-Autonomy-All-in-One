#!/usr/bin/env python3

import os
import unittest


PROJECT_ROOT = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "..")
)
PACKAGE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


class SharedArmEntrypointTest(unittest.TestCase):
    def read_project_file(self, relative_path):
        path = os.path.join(PROJECT_ROOT, relative_path)
        if not os.path.isfile(path):
            self.skipTest(
                "repository launchers are not mounted into this catkin container"
            )
        with open(path, "r", encoding="utf-8") as stream:
            return stream.read()

    def read_package_file(self, relative_path):
        with open(
            os.path.join(PACKAGE_ROOT, relative_path), "r", encoding="utf-8"
        ) as stream:
            return stream.read()

    def test_both_launchers_stream_the_shared_executor(self):
        for launcher in ("launch/sim.sh", "launch/real.sh"):
            with self.subTest(launcher=launcher):
                source = self.read_project_file(launcher)
                self.assertIn("ARM_EXECUTOR_HOST=", source)
                self.assertIn('"python3 -u - \\', source)
                self.assertIn('< "$ARM_EXECUTOR_HOST"', source)
                self.assertNotIn("request_arm_and_auto_takeoff", source)
                self.assertNotIn("prepare_native_takeoff", source)
                self.assertNotIn("flight_snapshot", source)

    def test_shared_executor_has_no_platform_branch(self):
        source = self.read_package_file("scripts/arm_executor.py")
        self.assertNotIn("SIM_", source)
        self.assertNotIn("REAL_", source)


if __name__ == "__main__":
    unittest.main()
