#!/usr/bin/env python3

import os
import unittest


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))


class SharedLocalizationGuardEntrypointTest(unittest.TestCase):
    def test_both_stacks_start_the_same_guard(self):
        expected = "rosrun sim2real_common localization_guard.py"
        for launcher in ("launch/sim.sh", "launch/real.sh"):
            with self.subTest(launcher=launcher):
                path = os.path.join(ROOT, launcher)
                if not os.path.exists(path):
                    self.skipTest(
                        "host launch wrappers are not copied into this container"
                    )
                with open(path, encoding="utf-8") as handle:
                    self.assertIn(expected, handle.read())


if __name__ == "__main__":
    unittest.main()
