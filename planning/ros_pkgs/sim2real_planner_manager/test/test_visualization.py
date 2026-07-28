#!/usr/bin/env python3

import struct
import unittest

from sim2real_planner_manager.visualization import (
    filter_point_records,
    fixed_bounds_edges,
)


class VisualizationTest(unittest.TestCase):
    def test_point_filter_preserves_records_and_removes_floor_and_nan(self):
        point_step = 16
        records = [
            struct.pack("<fffI", 1.0, 2.0, 0.2, 10),
            struct.pack("<fffI", 2.0, 3.0, 0.5, 20),
            struct.pack("<fffI", float("nan"), 4.0, 1.0, 30),
            struct.pack("<fffI", 3.0, 4.0, 3.3, 40),
        ]
        data, kept = filter_point_records(
            b"".join(records),
            width=4,
            height=1,
            point_step=point_step,
            row_step=4 * point_step,
            coordinate_specs={"x": (0, "f"), "y": (4, "f"), "z": (8, "f")},
            is_bigendian=False,
            min_z=0.36,
            max_z=3.2,
        )
        self.assertEqual(kept, 1)
        self.assertEqual(data, records[1])

    def test_point_filter_handles_organized_rows_and_padding(self):
        first = struct.pack(">ddd", 1.0, 2.0, 0.5)
        second = struct.pack(">ddd", 3.0, 4.0, 0.1)
        row_step = len(first) + 8
        data, kept = filter_point_records(
            first + b"\0" * 8 + second + b"\0" * 8,
            width=1,
            height=2,
            point_step=len(first),
            row_step=row_step,
            coordinate_specs={"x": (0, "d"), "y": (8, "d"), "z": (16, "d")},
            is_bigendian=True,
            min_z=0.36,
            max_z=3.2,
        )
        self.assertEqual(kept, 1)
        self.assertEqual(data, first)

    def test_fixed_bounds_has_twelve_edges(self):
        edges = fixed_bounds_edges((-15.0, -15.0, -1.0), (15.0, 15.0, 4.0))
        self.assertEqual(len(edges), 24)
        self.assertIn((-15.0, -15.0, -1.0), edges)
        self.assertIn((15.0, 15.0, 4.0), edges)
        with self.assertRaises(ValueError):
            fixed_bounds_edges((0.0, 0.0, 0.0), (0.0, 1.0, 1.0))

if __name__ == "__main__":
    unittest.main()
