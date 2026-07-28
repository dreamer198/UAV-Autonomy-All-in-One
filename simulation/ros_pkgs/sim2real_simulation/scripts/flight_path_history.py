#!/usr/bin/env python3
"""ROS-independent storage for one bounded, measured flight path."""

from collections import deque
import math


class FlightPathHistory:
    def __init__(self, min_distance, max_points):
        self.min_distance = float(min_distance)
        self.max_points = int(max_points)
        if not math.isfinite(self.min_distance) or self.min_distance <= 0.0:
            raise ValueError("min_distance must be finite and positive")
        if self.max_points < 2:
            raise ValueError("max_points must be at least two")

        self._points = deque(maxlen=self.max_points)
        self._armed = False
        self._have_state = False
        self._latest_stamp = None

    def set_armed(self, armed):
        """Update arm state and clear history at the start of each sortie."""

        armed = bool(armed)
        started_sortie = armed and (not self._have_state or not self._armed)
        self._have_state = True
        self._armed = armed
        if started_sortie:
            self.clear()
        return started_sortie

    def clear(self):
        self._points.clear()
        self._latest_stamp = None

    @property
    def armed(self):
        return self._armed

    def add(self, stamp, position):
        if not self._armed:
            return False
        stamp = float(stamp)
        point = tuple(float(value) for value in position)
        if (
            not math.isfinite(stamp)
            or stamp <= 0.0
            or len(point) != 3
            or not all(math.isfinite(value) for value in point)
        ):
            return False

        if self._latest_stamp is not None:
            if stamp < self._latest_stamp:
                self.clear()
            elif stamp == self._latest_stamp:
                return False
        self._latest_stamp = stamp

        if self._points:
            previous = self._points[-1]
            squared_distance = sum(
                (current - old) ** 2
                for current, old in zip(point, previous)
            )
            if squared_distance < self.min_distance * self.min_distance:
                return False

        self._points.append(point)
        return True

    def points(self):
        return list(self._points)
