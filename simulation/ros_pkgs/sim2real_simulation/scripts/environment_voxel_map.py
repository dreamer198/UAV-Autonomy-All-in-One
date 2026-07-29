#!/usr/bin/env python3
"""Small, ROS-independent persistent voxel map used only for visualization."""

import numpy as np


class PersistentVoxelMap:
    """Accumulate repeatedly observed points without changing planner maps."""

    def __init__(
        self,
        voxel_size,
        min_range,
        max_range,
        min_z,
        max_z,
        min_hits,
        max_voxels,
    ):
        self.voxel_size = float(voxel_size)
        self.min_range = float(min_range)
        self.max_range = float(max_range)
        self.min_z = float(min_z)
        self.max_z = float(max_z)
        self.min_hits = int(min_hits)
        self.max_voxels = int(max_voxels)

        if not np.isfinite(self.voxel_size) or self.voxel_size <= 0.0:
            raise ValueError("voxel_size must be finite and positive")
        if (
            not np.isfinite(self.min_range)
            or not np.isfinite(self.max_range)
            or self.min_range < 0.0
            or self.max_range <= self.min_range
        ):
            raise ValueError("range limits must be finite and increasing")
        if (
            not np.isfinite(self.min_z)
            or not np.isfinite(self.max_z)
            or self.max_z <= self.min_z
        ):
            raise ValueError("z limits must be finite and increasing")
        if self.min_hits < 1:
            raise ValueError("min_hits must be at least one")
        if self.max_voxels < 1:
            raise ValueError("max_voxels must be at least one")

        self._hits = {}
        self.total_input_points = 0
        self.total_accepted_points = 0
        self.total_dropped_voxels = 0

    def add(self, points, sensor_origin):
        """Filter and add one world-frame scan.

        Points at the sensor's maximum return distance are deliberately
        rejected. Gazebo's MID-360 represents a no-return ray as an endpoint,
        and accumulating those endpoints creates phantom colored shells.
        """

        points = np.asarray(points, dtype=np.float32)
        sensor_origin = np.asarray(sensor_origin, dtype=np.float32)
        if points.size == 0:
            return 0
        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError("points must have shape (N, 3)")
        if sensor_origin.shape != (3,) or not np.all(np.isfinite(sensor_origin)):
            raise ValueError("sensor_origin must contain three finite values")

        self.total_input_points += int(points.shape[0])
        finite_mask = np.all(np.isfinite(points), axis=1)
        points = points[finite_mask]
        if points.size == 0:
            return 0

        offset = points - sensor_origin
        squared_range = np.einsum("ij,ij->i", offset, offset)
        range_mask = (
            (squared_range >= self.min_range * self.min_range)
            & (squared_range <= self.max_range * self.max_range)
        )
        z_mask = (points[:, 2] >= self.min_z) & (points[:, 2] <= self.max_z)
        points = points[range_mask & z_mask]
        if points.size == 0:
            return 0

        voxel_keys = np.floor(points / self.voxel_size).astype(np.int32)
        voxel_keys = np.unique(voxel_keys, axis=0)
        self.total_accepted_points += int(voxel_keys.shape[0])

        added = 0
        for raw_key in voxel_keys:
            key = (int(raw_key[0]), int(raw_key[1]), int(raw_key[2]))
            previous_hits = self._hits.get(key)
            if previous_hits is not None:
                if previous_hits < self.min_hits:
                    self._hits[key] = previous_hits + 1
                continue
            if len(self._hits) >= self.max_voxels:
                self.total_dropped_voxels += 1
                continue
            self._hits[key] = 1
            added += 1
        return added

    @property
    def voxel_count(self):
        return len(self._hits)

    @property
    def visible_voxel_count(self):
        return sum(1 for hits in self._hits.values() if hits >= self.min_hits)

    def points(self):
        """Return confirmed voxel centers as an N x 3 float32 array."""

        visible_keys = [
            key for key, hits in self._hits.items() if hits >= self.min_hits
        ]
        if not visible_keys:
            return np.empty((0, 3), dtype=np.float32)
        keys = np.asarray(visible_keys, dtype=np.float32)
        return (keys + 0.5) * self.voxel_size
