#!/usr/bin/env python3

import math
import os
import unittest

import yaml


class DiffPlannerConfigTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        config_path = os.path.join(
            os.path.dirname(__file__), "..", "config", "planner.yaml"
        )
        with open(config_path, "r", encoding="utf-8") as config_file:
            cls.config = yaml.safe_load(config_file)
        adapter_config_path = os.path.join(
            os.path.dirname(__file__), "..", "config", "diff.yaml"
        )
        with open(
            adapter_config_path, "r", encoding="utf-8"
        ) as adapter_config_file:
            cls.adapter_config = yaml.safe_load(adapter_config_file)

    def test_mid360_map_coverage(self):
        grid_map = self.config["grid_map"]
        resolution = float(grid_map["resolution"])
        self.assertGreater(resolution, 0.0)

        for axis in ("x", "y"):
            half_range = float(grid_map["local_update_range_{}".format(axis)])
            quantized_half_range = math.ceil(half_range / resolution) * resolution
            self.assertGreaterEqual(quantized_half_range, 5.5)

        self.assertIs(grid_map["visualize_all_directions"], True)

        visualization_period = float(grid_map["visualization_period"])
        self.assertTrue(math.isfinite(visualization_period))
        self.assertGreater(visualization_period, 0.0)
        self.assertGreaterEqual(visualization_period, 0.5)

    def test_stuck_detection_allows_startup_and_reversal_transients(self):
        fsm = self.config["fsm"]
        progress_threshold = float(fsm["stuck_progress_threshold"])
        timeout = float(fsm["stuck_timeout"])
        goal_tolerance = float(fsm["goal_position_tolerance"])

        self.assertTrue(math.isfinite(progress_threshold))
        self.assertTrue(math.isfinite(timeout))
        self.assertTrue(math.isfinite(goal_tolerance))
        self.assertEqual(progress_threshold, 0.1)
        self.assertEqual(timeout, 5.0)
        self.assertEqual(goal_tolerance, 0.35)

    def test_replan_failures_use_a_timed_retry_window(self):
        fsm = self.config["fsm"]
        retry_interval = float(fsm["replan_retry_interval"])
        failure_timeout = float(fsm["replan_failure_timeout"])
        planning_timeout = float(
            self.adapter_config["backend"]["planning_timeout"]
        )

        self.assertTrue(math.isfinite(retry_interval))
        self.assertTrue(math.isfinite(failure_timeout))
        self.assertGreater(retry_interval, 0.0)
        self.assertGreater(failure_timeout, retry_interval)
        self.assertEqual(retry_interval, 0.1)
        self.assertEqual(failure_timeout, 1.0)
        self.assertTrue(math.isfinite(planning_timeout))
        self.assertEqual(planning_timeout, 10.0)

    def test_inflation_matches_current_airframe_baseline(self):
        grid_map = self.config["grid_map"]
        resolution = float(grid_map["resolution"])
        inflation = float(grid_map["obstacles_inflation"])
        inflation_cells = math.ceil((inflation - 1e-5) / resolution)
        quantized_inflation = inflation_cells * resolution

        # A square airframe with a 0.65 m diagonal has a 0.325 m
        # circumradius. The requested 0.33 m inflation deliberately keeps
        # only 0.005 m of nominal clearance and quantizes to exactly 3 cells.
        required_radius = 0.65 / 2.0
        self.assertEqual(inflation_cells, 3)
        self.assertAlmostEqual(resolution, 0.11)
        self.assertAlmostEqual(inflation, 0.33)
        self.assertAlmostEqual(quantized_inflation, 0.33)
        self.assertGreaterEqual(quantized_inflation, required_radius)

    def test_adapter_stop_is_recoverable_without_weakening_mandatory_stop(self):
        package_root = os.path.realpath(
            os.path.join(os.path.dirname(__file__), "..")
        )
        launch_path = os.path.join(
            package_root, "launch", "diff_backend.launch"
        )
        fsm_path = os.path.abspath(
            os.path.join(
                package_root,
                "..",
                "..",
                "..",
                "third_party",
                "Diff-Planner-PX4",
                "src",
                "diff_planner",
                "plan_manage",
                "src",
                "diff_replan_fsm.cpp",
            )
        )
        with open(launch_path, "r", encoding="utf-8") as launch_file:
            launch = launch_file.read()
        with open(fsm_path, "r", encoding="utf-8") as fsm_file:
            fsm = fsm_file.read()

        self.assertIn(
            'name="backend/native_stop_topic" '
            'value="/planning/backends/$(arg backend_id)/native/recoverable_stop"',
            launch,
        )
        self.assertIn("recoverableStopCallback", fsm)
        mandatory_callback = fsm.split(
            "void DiffReplanFSM::mandatoryStopCallback", 1
        )[1].split(
            "void DiffReplanFSM::recoverableStopCallback", 1
        )[0]
        self.assertIn("enable_fail_safe_ = false", mandatory_callback)


if __name__ == "__main__":
    unittest.main()
