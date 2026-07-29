#!/usr/bin/env python3

import unittest

from sim2real_planner_manager.rate_monitor import (
    observed_rate,
    rate_sample_time,
)


class RateMonitorTest(unittest.TestCase):
    def test_simulation_rate_uses_ros_message_time(self):
        # A 100 Hz simulated stream delivered at 75 Hz wall time is still a
        # 100 Hz control stream in the simulated vehicle's clock domain.
        samples = [
            rate_sample_time("simulation", index / 75.0, index / 100.0 + 1.0)
            for index in range(101)
        ]
        self.assertAlmostEqual(observed_rate(samples, 2.0, 1.0), 100.0)

    def test_real_rate_uses_monotonic_receipt_time(self):
        samples = [
            rate_sample_time("real", index / 75.0, index / 100.0 + 1.0)
            for index in range(76)
        ]
        self.assertAlmostEqual(observed_rate(samples, 1.0, 1.0), 75.0)

    def test_simulation_rejects_invalid_message_clock(self):
        with self.assertRaises(ValueError):
            rate_sample_time("simulation", 1.0, 0.0)

    def test_rate_waits_for_full_window(self):
        self.assertIsNone(observed_rate([1.0, 1.5], 1.5, 1.0))


if __name__ == "__main__":
    unittest.main()
