#!/usr/bin/env python3

import struct
import unittest

from sim2real_planner_manager.visualization import (
    finite_point_records,
    fixed_bounds_edges,
)


class VisualizationTest(unittest.TestCase):
    def test_point_normalization_preserves_all_finite_heights(self):
        point_step = 16
        records = [
            struct.pack("<fffI", 1.0, 2.0, 0.2, 10),
            struct.pack("<fffI", 2.0, 3.0, 0.5, 20),
            struct.pack("<fffI", float("nan"), 4.0, 1.0, 30),
            struct.pack("<fffI", 3.0, 4.0, 3.3, 40),
        ]
        data, kept = finite_point_records(
            b"".join(records),
            width=4,
            height=1,
            point_step=point_step,
            row_step=4 * point_step,
            coordinate_specs={"x": (0, "f"), "y": (4, "f"), "z": (8, "f")},
            is_bigendian=False,
        )
        self.assertEqual(kept, 3)
        self.assertEqual(data, records[0] + records[1] + records[3])

    def test_point_filter_handles_organized_rows_and_padding(self):
        first = struct.pack(">ddd", 1.0, 2.0, 0.5)
        second = struct.pack(">ddd", 3.0, 4.0, 0.1)
        row_step = len(first) + 8
        data, kept = finite_point_records(
            first + b"\0" * 8 + second + b"\0" * 8,
            width=1,
            height=2,
            point_step=len(first),
            row_step=row_step,
            coordinate_specs={"x": (0, "d"), "y": (8, "d"), "z": (16, "d")},
            is_bigendian=True,
        )
        self.assertEqual(kept, 2)
        self.assertEqual(data, first + second)

    def test_fixed_bounds_has_twelve_edges(self):
        edges = fixed_bounds_edges((-15.0, -15.0, -1.0), (15.0, 15.0, 4.0))
        self.assertEqual(len(edges), 24)
        self.assertIn((-15.0, -15.0, -1.0), edges)
        self.assertIn((15.0, 15.0, 4.0), edges)
        with self.assertRaises(ValueError):
            fixed_bounds_edges((0.0, 0.0, 0.0), (0.0, 1.0, 1.0))

if __name__ == "__main__":
    unittest.main()
