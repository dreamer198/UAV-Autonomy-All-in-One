#!/usr/bin/env python3
"""Emit one JSON telemetry stream for the local Qt ground station.

This helper deliberately runs inside the simulation container so the Qt
application does not need ROS Noetic Python packages on the host.
"""

import argparse
import json
import math
import sys
import threading

import rospy
from mavros_msgs.msg import ExtendedState, State
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, NavSatFix


def finite(value, fallback=0.0):
    try:
        result = float(value)
    except (TypeError, ValueError):
        return float(fallback)
    return result if math.isfinite(result) else float(fallback)


def euler_degrees(quaternion):
    x = finite(quaternion.x)
    y = finite(quaternion.y)
    z = finite(quaternion.z)
    w = finite(quaternion.w, 1.0)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-9:
        return 0.0, 0.0, 0.0
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    sin_roll = 2.0 * (w * x + y * z)
    cos_roll = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sin_roll, cos_roll)
    sin_pitch = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sin_pitch)
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(sin_yaw, cos_yaw)
    scale = 180.0 / math.pi
    return roll * scale, pitch * scale, yaw * scale


class GroundStationTelemetry:
    def __init__(self):
        self._lock = threading.Lock()
        self._state = None
        self._extended_state = None
        self._odometry = None
        self._battery = None
        self._gps = None

        rospy.Subscriber("/mavros/state", State, self._on_state, queue_size=1)
        rospy.Subscriber(
            "/mavros/extended_state",
            ExtendedState,
            self._on_extended_state,
            queue_size=1,
        )
        rospy.Subscriber(
            "/localization/odom", Odometry, self._on_odometry, queue_size=1
        )
        rospy.Subscriber(
            "/mavros/battery", BatteryState, self._on_battery, queue_size=1
        )
        rospy.Subscriber(
            "/mavros/global_position/global",
            NavSatFix,
            self._on_gps,
            queue_size=1,
        )
        self._timer = rospy.Timer(rospy.Duration(0.25), self._publish)

    def _on_state(self, message):
        with self._lock:
            self._state = message

    def _on_extended_state(self, message):
        with self._lock:
            self._extended_state = message

    def _on_odometry(self, message):
        with self._lock:
            self._odometry = message

    def _on_battery(self, message):
        with self._lock:
            self._battery = message

    def _on_gps(self, message):
        with self._lock:
            self._gps = message

    def _publish(self, _event):
        with self._lock:
            state = self._state
            extended_state = self._extended_state
            odometry = self._odometry
            battery = self._battery
            gps = self._gps
        if state is None or odometry is None:
            return

        position = odometry.pose.pose.position
        velocity = odometry.twist.twist.linear
        roll, pitch, yaw = euler_degrees(odometry.pose.pose.orientation)
        speed = math.sqrt(
            finite(velocity.x) ** 2
            + finite(velocity.y) ** 2
            + finite(velocity.z) ** 2
        )

        percentage = 100.0
        voltage = 0.0
        current = 0.0
        remaining_time = 0
        if battery is not None:
            raw_percentage = finite(battery.percentage, 1.0)
            percentage = raw_percentage * 100.0 if raw_percentage <= 1.0 else raw_percentage
            voltage = finite(battery.voltage)
            current = finite(battery.current)

        latitude = 47.397742
        longitude = 8.545594
        altitude_amsl = finite(position.z)
        gps_fix_type = 0
        if gps is not None:
            latitude = finite(gps.latitude, latitude)
            longitude = finite(gps.longitude, longitude)
            altitude_amsl = finite(gps.altitude, altitude_amsl)
            gps_fix_type = 3 if int(gps.status.status) >= 0 else 0

        landed = False
        if extended_state is not None:
            landed = (
                int(extended_state.landed_state)
                == int(ExtendedState.LANDED_STATE_ON_GROUND)
            )

        payload = {
            "type": "telemetry",
            "connected": bool(state.connected),
            "armed": bool(state.armed),
            "landed": bool(landed),
            "flight_mode": str(state.mode),
            "latitude": latitude,
            "longitude": longitude,
            "altitude_relative": finite(position.z),
            "altitude_amsl": altitude_amsl,
            "speed": speed,
            "heading": yaw % 360.0,
            "roll": roll,
            "pitch": pitch,
            "yaw": yaw,
            "battery": max(0.0, min(100.0, percentage)),
            "voltage": voltage,
            "current": current,
            "remaining_time": remaining_time,
            "gps_fix_type": gps_fix_type,
        }
        try:
            print(json.dumps(payload, ensure_ascii=False, allow_nan=False), flush=True)
        except (TypeError, ValueError) as exc:
            print("telemetry serialization failed: {}".format(exc), file=sys.stderr)


def main():
    parser = argparse.ArgumentParser()
    # The token is intentionally retained in /proc/cmdline so the launching Qt
    # process can remove only its own helper after a docker-exec disconnect.
    parser.add_argument("--session-token", required=True)
    parser.parse_args(rospy.myargv()[1:])
    rospy.init_node("swarm_ground_station_telemetry", anonymous=True)
    GroundStationTelemetry()
    rospy.spin()


if __name__ == "__main__":
    main()
