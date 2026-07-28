"""Duck-typed validation helpers shared by ROS callbacks and unit tests."""

from __future__ import annotations

import math
from typing import Iterable, Tuple


def _finite(values: Iterable[float]) -> bool:
    try:
        return all(math.isfinite(float(value)) for value in values)
    except (TypeError, ValueError):
        return False


def validate_goal_pose(message, expected_frame: str = "world") -> Tuple[bool, str, bool]:
    """Return (valid, reason, constrain_yaw) for a PoseStamped-like object."""
    if message.header.frame_id != expected_frame:
        return (
            False,
            "goal frame must be {!r}".format(expected_frame),
            False,
        )
    if hasattr(message.header.stamp, "is_zero") and message.header.stamp.is_zero():
        return False, "goal stamp must be non-zero", False
    position = message.pose.position
    orientation = message.pose.orientation
    values = (
        position.x,
        position.y,
        position.z,
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w,
    )
    if not _finite(values):
        return False, "goal contains non-finite values", False
    norm_sq = (
        orientation.x * orientation.x
        + orientation.y * orientation.y
        + orientation.z * orientation.z
        + orientation.w * orientation.w
    )
    if norm_sq <= 1.0e-12:
        return True, "", False
    norm = math.sqrt(norm_sq)
    if abs(norm - 1.0) > 1.0e-3:
        return False, "goal quaternion must be unit length or exactly zero", False
    if abs(orientation.x) > 1.0e-3 or abs(orientation.y) > 1.0e-3:
        return False, "goal orientation may constrain yaw only", False
    return True, "", True


def validate_trajectory_point(point) -> Tuple[bool, str]:
    """Validate the exact controller wire shape of one trajectory point."""
    if len(point.transforms) != 1:
        return False, "command must contain exactly one transform"
    if len(point.velocities) != 1:
        return False, "command must contain exactly one velocity"
    if len(point.accelerations) not in (0, 1):
        return False, "command may contain at most one acceleration"

    transform = point.transforms[0]
    velocity = point.velocities[0]
    values = [
        transform.translation.x,
        transform.translation.y,
        transform.translation.z,
        transform.rotation.x,
        transform.rotation.y,
        transform.rotation.z,
        transform.rotation.w,
        velocity.linear.x,
        velocity.linear.y,
        velocity.linear.z,
        velocity.angular.x,
        velocity.angular.y,
        velocity.angular.z,
    ]
    if point.accelerations:
        acceleration = point.accelerations[0]
        values.extend(
            [
                acceleration.linear.x,
                acceleration.linear.y,
                acceleration.linear.z,
                acceleration.angular.x,
                acceleration.angular.y,
                acceleration.angular.z,
            ]
        )
    if hasattr(point.time_from_start, "to_sec"):
        values.append(point.time_from_start.to_sec())
    if not _finite(values):
        return False, "command contains non-finite values"

    rotation = transform.rotation
    norm_sq = (
        rotation.x * rotation.x
        + rotation.y * rotation.y
        + rotation.z * rotation.z
        + rotation.w * rotation.w
    )
    if norm_sq <= 1.0e-12 or abs(math.sqrt(norm_sq) - 1.0) > 1.0e-3:
        return False, "command quaternion must be unit length"
    return True, ""


def validate_fixed_bounds(capabilities) -> Tuple[bool, str]:
    values = (
        capabilities.max_velocity,
        capabilities.max_acceleration,
        capabilities.map_min.x,
        capabilities.map_min.y,
        capabilities.map_min.z,
        capabilities.map_max.x,
        capabilities.map_max.y,
        capabilities.map_max.z,
    )
    if not _finite(values):
        return False, "capabilities contain non-finite numeric values"
    if capabilities.max_velocity <= 0.0 or capabilities.max_acceleration <= 0.0:
        return False, "capability dynamics limits must be positive"
    if capabilities.has_fixed_map_bounds and not (
        capabilities.map_min.x < capabilities.map_max.x
        and capabilities.map_min.y < capabilities.map_max.y
        and capabilities.map_min.z < capabilities.map_max.z
    ):
        return False, "fixed map bounds must be strictly increasing"
    return True, ""


def status_order_is_newer(
    stamp_sec: float,
    sequence: int,
    previous_stamp_sec: float,
    previous_sequence: int,
) -> bool:
    """Accept coarse-clock peers when their ROS Header sequence advances."""
    if not _finite((stamp_sec, previous_stamp_sec)):
        return False
    if (
        not isinstance(sequence, int)
        or isinstance(sequence, bool)
        or not isinstance(previous_sequence, int)
        or isinstance(previous_sequence, bool)
        or sequence < 0
        or previous_sequence < 0
    ):
        return False
    return bool(
        stamp_sec > previous_stamp_sec
        or (
            stamp_sec == previous_stamp_sec
            and sequence > previous_sequence
        )
    )


def validate_command_mode(mode, point, hold_mode: int = 1) -> Tuple[bool, str]:
    """Require HOLD to be a true stationary setpoint, not a relabeled motion."""
    if mode != hold_mode:
        return True, ""
    velocity = point.velocities[0]
    values = [
        velocity.linear.x,
        velocity.linear.y,
        velocity.linear.z,
        velocity.angular.x,
        velocity.angular.y,
        velocity.angular.z,
    ]
    if point.accelerations:
        acceleration = point.accelerations[0]
        values.extend(
            [
                acceleration.linear.x,
                acceleration.linear.y,
                acceleration.linear.z,
                acceleration.angular.x,
                acceleration.angular.y,
                acceleration.angular.z,
            ]
        )
    if not _finite(values) or any(abs(float(value)) > 1.0e-6 for value in values):
        return False, "HOLD command must have zero velocity and acceleration"
    return True, ""


def backend_status_allows_new_goal(
    status,
    *,
    received_at: float,
    now: float,
    timeout: float,
    allowed_states,
) -> bool:
    """Separate backend health for preemption from the initial READY state."""
    return bool(
        status is not None
        and status.state in allowed_states
        and status.odom_ready
        and status.map_ready
        and _finite((received_at, now, timeout))
        and timeout > 0.0
        and received_at > 0.0
        and 0.0 <= now - received_at <= timeout
    )
