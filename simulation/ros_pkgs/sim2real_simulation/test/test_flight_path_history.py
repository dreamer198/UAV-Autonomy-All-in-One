#!/usr/bin/env python3

import importlib.util
import os
import unittest


def load_history_module():
    path = os.path.join(
        os.path.dirname(__file__),
        "..",
        "scripts",
        "flight_path_history.py",
    )
    spec = importlib.util.spec_from_file_location("flight_path_history", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class FlightPathHistoryTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = load_history_module()

    def make_history(self, **overrides):
        options = {"min_distance": 0.1, "max_points": 4}
        options.update(overrides)
        return self.module.FlightPathHistory(**options)

    def test_only_records_armed_motion_and_retains_after_disarm(self):
        history = self.make_history()
        self.assertFalse(history.add(1.0, (0.0, 0.0, 1.0)))
        self.assertTrue(history.set_armed(True))
        self.assertTrue(history.add(1.0, (0.0, 0.0, 1.0)))
        self.assertFalse(history.add(2.0, (0.02, 0.0, 1.0)))
        self.assertTrue(history.add(3.0, (0.2, 0.0, 1.0)))
        history.set_armed(False)
        self.assertFalse(history.add(4.0, (1.0, 0.0, 1.0)))
        self.assertEqual(
            history.points(), [(0.0, 0.0, 1.0), (0.2, 0.0, 1.0)]
        )

    def test_new_sortie_clears_old_path(self):
        history = self.make_history()
        history.set_armed(True)
        history.add(1.0, (0.0, 0.0, 1.0))
        history.set_armed(False)
        self.assertTrue(history.set_armed(True))
        self.assertEqual(history.points(), [])

    def test_clock_rewind_starts_a_new_path(self):
        history = self.make_history()
        history.set_armed(True)
        history.add(10.0, (0.0, 0.0, 1.0))
        history.add(11.0, (1.0, 0.0, 1.0))
        self.assertTrue(history.add(2.0, (2.0, 0.0, 1.0)))
        self.assertEqual(history.points(), [(2.0, 0.0, 1.0)])

    def test_history_is_bounded(self):
        history = self.make_history(max_points=2)
        history.set_armed(True)
        for index in range(3):
            history.add(index + 1.0, (float(index), 0.0, 1.0))
        self.assertEqual(
            history.points(), [(1.0, 0.0, 1.0), (2.0, 0.0, 1.0)]
        )

    def test_invalid_configuration_is_rejected(self):
        for override in (
            {"min_distance": 0.0},
            {"min_distance": float("nan")},
            {"max_points": 1},
        ):
            with self.subTest(override=override):
                with self.assertRaises(ValueError):
                    self.make_history(**override)


if __name__ == "__main__":
    unittest.main()
