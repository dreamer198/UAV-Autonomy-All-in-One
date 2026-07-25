#!/usr/bin/env python3

import math
import os
import unittest

import yaml


class PlannerConfigTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        config_path = os.path.join(
            os.path.dirname(__file__), "..", "config", "planner.yaml"
        )
        with open(config_path, "r", encoding="utf-8") as config_file:
            cls.config = yaml.safe_load(config_file)

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

    def test_goal_distance_guard_is_enabled(self):
        max_goal_distance = float(self.config["fsm"]["max_goal_distance"])
        self.assertTrue(math.isfinite(max_goal_distance))
        self.assertEqual(max_goal_distance, 200.0)

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


if __name__ == "__main__":
    unittest.main()
