#!/usr/bin/env python3

import os
import unittest


PROJECT_ROOT = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "..")
)


class SimulationBuildRetryTest(unittest.TestCase):
    def read_launcher(self):
        path = os.path.join(PROJECT_ROOT, "launch", "sim.sh")
        if not os.path.isfile(path):
            self.skipTest(
                "repository launcher is not mounted into this catkin container"
            )
        with open(path, "r", encoding="utf-8") as stream:
            return stream.read()

    def test_retries_transient_compiler_crashes_serially(self):
        source = self.read_launcher()
        self.assertIn("internal compiler error:", source)
        self.assertIn("GCC crashed while compiling the overlay", source)
        self.assertIn("catkin build --no-status -j1 -p1", source)

    def test_does_not_retry_ordinary_build_failures(self):
        source = self.read_launcher()
        self.assertIn('exit "$build_status"', source)


if __name__ == "__main__":
    unittest.main()
