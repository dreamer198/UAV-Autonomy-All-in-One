#!/usr/bin/env python3
"""Translate Diff-Planner's private ROS API to the common planner API."""

import copy
import math
import os
import threading
import time


WORLD_FRAME = "world"
BASE_FRAME = "base_link"


def finite(values):
    return all(math.isfinite(float(value)) for value in values)


def normalized_frame(frame_id):
    return str(frame_id or "").lstrip("/")


def valid_stamp(stamp):
    try:
        return int(stamp.secs) != 0 or int(stamp.nsecs) != 0
    except (AttributeError, TypeError, ValueError):
        return False


def measurement_stamp_is_current(
    stamp,
    now_seconds,
    maximum_age,
    previous_seconds=0.0,
    future_tolerance=0.1,
):
    try:
        stamp_seconds = float(stamp.secs) + float(stamp.nsecs) * 1.0e-9
        age = float(now_seconds) - stamp_seconds
        return (
            valid_stamp(stamp)
            and math.isfinite(stamp_seconds)
            and math.isfinite(age)
            and math.isfinite(float(maximum_age))
            and math.isfinite(float(future_tolerance))
            and float(maximum_age) > 0.0
            and float(future_tolerance) >= 0.0
            and stamp_seconds > float(previous_seconds)
            and -float(future_tolerance) <= age <= float(maximum_age)
        )
    except (AttributeError, TypeError, ValueError):
        return False


def quaternion_is_valid(quaternion, allow_zero=False):
    values = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
    if not finite(values):
        return False
    norm_squared = sum(float(value) ** 2 for value in values)
    if allow_zero and norm_squared <= 1.0e-12:
        return True
    return 0.99 <= norm_squared <= 1.01


def quaternion_yaw(quaternion):
    sin_yaw = 2.0 * (
        float(quaternion.w) * float(quaternion.z)
        + float(quaternion.x) * float(quaternion.y)
    )
    cos_yaw = 1.0 - 2.0 * (
        float(quaternion.y) ** 2 + float(quaternion.z) ** 2
    )
    return math.atan2(sin_yaw, cos_yaw)


def wrapped_angle_error(target, current):
    return math.atan2(
        math.sin(float(target) - float(current)),
        math.cos(float(target) - float(current)),
    )


def validate_pose_goal(planner_goal, ground, ceiling, inflation):
    """Return an empty string for a valid common goal, otherwise the reason."""
    if planner_goal.action == planner_goal.CANCEL:
        return ""
    if planner_goal.action != planner_goal.PLAN:
        return "unknown PlannerGoal action"
    if not planner_goal.session_id:
        return "session_id is empty"
    if int(planner_goal.goal_id) <= 0:
        return "goal_id must be positive"

    goal = planner_goal.goal
    if normalized_frame(goal.header.frame_id) != WORLD_FRAME:
        return "goal frame_id must be world"
    if not valid_stamp(goal.header.stamp):
        return "goal measurement timestamp must be non-zero"
    position = goal.pose.position
    if not finite((position.x, position.y, position.z)):
        return "goal position contains NaN or Inf"
    if not finite((ground, ceiling, inflation)) or inflation < 0.0:
        return "Diff vertical-map configuration is invalid"
    minimum = float(ground) + float(inflation)
    maximum = float(ceiling) - float(inflation)
    if minimum >= maximum:
        return "Diff vertical-map clearance interval is empty"
    if not minimum < float(position.z) < maximum:
        return "goal z must be inside ({:.3f}, {:.3f})".format(
            minimum, maximum
        )
    if planner_goal.constrain_yaw:
        if not quaternion_is_valid(goal.pose.orientation):
            return "constrained goal yaw requires a finite unit quaternion"
    elif not quaternion_is_valid(goal.pose.orientation, allow_zero=True):
        return "goal orientation is invalid"
    return ""


def valid_odometry(message):
    pose = message.pose.pose
    twist = message.twist.twist
    return (
        valid_stamp(message.header.stamp)
        and normalized_frame(message.header.frame_id) == WORLD_FRAME
        and normalized_frame(message.child_frame_id) == BASE_FRAME
        and finite(
            (
                pose.position.x,
                pose.position.y,
                pose.position.z,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
                twist.linear.x,
                twist.linear.y,
                twist.linear.z,
                twist.angular.x,
                twist.angular.y,
                twist.angular.z,
            )
        )
        and quaternion_is_valid(pose.orientation)
    )


def valid_cloud_header(message):
    expected_size = int(message.row_step) * int(message.height)
    return (
        valid_stamp(message.header.stamp)
        and normalized_frame(message.header.frame_id) == WORLD_FRAME
        and int(message.width) * int(message.height) > 0
        and int(message.point_step) > 0
        and int(message.row_step) > 0
        and expected_size <= len(message.data)
    )


def valid_native_trajectory(message):
    piece_count = len(message.duration)
    coefficient_count = piece_count * (int(message.order) + 1)
    values = (
        list(message.duration)
        + list(message.coef_x)
        + list(message.coef_y)
        + list(message.coef_z)
        + list(message.goal_position)
        + [message.goal_yaw]
    )
    return (
        int(message.traj_id) > 0
        and int(message.order) == 5
        and piece_count > 0
        and len(message.coef_x) == coefficient_count
        and len(message.coef_y) == coefficient_count
        and len(message.coef_z) == coefficient_count
        and valid_stamp(message.start_time)
        and finite(values)
        and all(float(duration) > 0.0 for duration in message.duration)
    )


def valid_native_command(message):
    return (
        int(message.trajectory_id) > 0
        and finite(
            (
                message.position.x,
                message.position.y,
                message.position.z,
                message.velocity.x,
                message.velocity.y,
                message.velocity.z,
                message.acceleration.x,
                message.acceleration.y,
                message.acceleration.z,
                message.jerk.x,
                message.jerk.y,
                message.jerk.z,
                message.yaw,
                message.yaw_dot,
            )
        )
    )


class DiffBackendAdapter:
    def __init__(self):
        import rospy
        from geometry_msgs.msg import Transform, Twist
        from nav_msgs.msg import Odometry
        from quadrotor_msgs.msg import PositionCommand
        from sensor_msgs.msg import PointCloud2
        from sim2real_planning_msgs.msg import (
            PlannerCapabilities,
            PlannerCommand,
            PlannerGoal,
            PlannerStatus,
        )
        from sim2real_planning_msgs.srv import ValidateGoal, ValidateGoalResponse
        from std_msgs.msg import Empty, Header
        from tf.transformations import quaternion_from_euler
        from traj_utils.msg import PolyTraj

        self.rospy = rospy
        self.Transform = Transform
        self.Twist = Twist
        self.PlannerCapabilities = PlannerCapabilities
        self.PlannerCommand = PlannerCommand
        self.PlannerGoal = PlannerGoal
        self.PlannerStatus = PlannerStatus
        self.ValidateGoalResponse = ValidateGoalResponse
        self.Header = Header
        self.quaternion_from_euler = quaternion_from_euler

        self.backend_id = rospy.get_param("~backend/backend_id", "diff")
        if self.backend_id != "diff":
            raise ValueError("Diff adapter backend_id must be 'diff'")
        self.profile = rospy.get_param("~backend/profile", "local")
        if self.profile != "local":
            raise ValueError("Diff adapter only supports the local profile")
        self.runtime_mode = rospy.get_param("~backend/runtime_mode", "")
        environment_mode = os.environ.get("SIM2REAL_RUNTIME_MODE", "")
        if (
            self.runtime_mode not in {"simulation", "real"}
            or environment_mode not in {"simulation", "real"}
            or self.runtime_mode != environment_mode
        ):
            raise ValueError(
                "Diff runtime_mode must exactly match SIM2REAL_RUNTIME_MODE"
            )
        self.api_version = rospy.get_param(
            "~backend/api_version", "sim2real.planner/v1"
        )
        self.odom_timeout = self._positive_param("odom_timeout", 0.5)
        self.cloud_timeout = self._positive_param("cloud_timeout", 1.0)
        self.heartbeat_timeout = self._positive_param(
            "heartbeat_timeout", 0.5
        )
        self.startup_grace = self._positive_param("startup_grace", 10.0)
        self.planning_timeout = self._positive_param(
            "planning_timeout", 10.0
        )
        self.status_rate = self._positive_param("status_rate", 20.0)
        self.goal_tolerance = self._positive_param(
            "goal_position_tolerance", 0.35
        )
        self.reached_velocity_tolerance = self._positive_param(
            "reached_velocity_tolerance", 0.2
        )
        self.reached_yaw_tolerance = math.radians(
            self._positive_param("reached_yaw_tolerance_deg", 5.0)
        )
        self.reached_yaw_rate_tolerance = math.radians(
            self._positive_param(
                "reached_yaw_rate_tolerance_deg_s", 10.0
            )
        )
        self.reached_hold_time = self._positive_param(
            "reached_hold_time", 0.5
        )
        self.virtual_ground = self._finite_param("virtual_ground", 0.1)
        self.virtual_ceil = self._finite_param("virtual_ceil", 3.0)
        self.obstacle_inflation = self._finite_param(
            "obstacle_inflation", 0.33
        )
        self.max_velocity = self._positive_param("max_velocity", 0.5)
        self.max_acceleration = self._positive_param(
            "max_acceleration", 0.8
        )
        if (
            self.obstacle_inflation < 0.0
            or self.virtual_ground + self.obstacle_inflation
            >= self.virtual_ceil - self.obstacle_inflation
        ):
            raise ValueError("Diff vertical-map clearance is invalid")

        self.lock = threading.RLock()
        self.started_at = time.monotonic()
        self.ever_ready = False
        self.last_odom_at = 0.0
        self.last_cloud_at = 0.0
        self.last_heartbeat_at = 0.0
        self.last_odom_stamp = 0.0
        self.last_cloud_stamp = 0.0
        self.odom = None
        self.session_id = ""
        self.goal_id = 0
        self.goal = None
        self.constrain_yaw = False
        self.native_goal_stamp = None
        self.trajectory_id = 0
        self.native_armable = False
        self.cancelled = False
        self.reached_at = None
        self.planning_started_at = 0.0
        self.fault_reason = ""

        odom_topic = rospy.get_param(
            "~backend/odom_topic", "/localization/odom"
        )
        cloud_topic = rospy.get_param(
            "~backend/cloud_topic", "/localization/cloud_registered"
        )
        native_goal_topic = rospy.get_param(
            "~backend/native_goal_topic",
            "/planning/backends/diff/native/goal",
        )
        native_stop_topic = rospy.get_param(
            "~backend/native_stop_topic",
            "/planning/backends/diff/native/recoverable_stop",
        )
        native_trajectory_topic = rospy.get_param(
            "~backend/native_trajectory_topic",
            "/planning/backends/diff/native/trajectory",
        )
        native_command_topic = rospy.get_param(
            "~backend/native_command_topic",
            "/planning/backends/diff/native/position_command",
        )
        native_heartbeat_topic = rospy.get_param(
            "~backend/native_heartbeat_topic",
            "/planning/backends/diff/native/heartbeat",
        )
        native_odom_topic = rospy.get_param(
            "~backend/native_odom_topic",
            "/planning/backends/diff/native/odom_world",
        )
        native_cloud_topic = rospy.get_param(
            "~backend/native_cloud_topic",
            "/planning/backends/diff/native/cloud_world",
        )

        self.goal_pub = rospy.Publisher(
            native_goal_topic, self._pose_type(), queue_size=1
        )
        self.stop_pub = rospy.Publisher(
            native_stop_topic, Header, queue_size=1
        )
        self.command_pub = rospy.Publisher(
            "command", PlannerCommand, queue_size=10
        )
        self.native_odom_pub = rospy.Publisher(
            native_odom_topic, Odometry, queue_size=2
        )
        self.native_cloud_pub = rospy.Publisher(
            native_cloud_topic, PointCloud2, queue_size=2
        )
        self.status_pub = rospy.Publisher(
            "status", PlannerStatus, queue_size=10, latch=True
        )
        self.capabilities_pub = rospy.Publisher(
            "capabilities", PlannerCapabilities, queue_size=1, latch=True
        )

        rospy.Subscriber("goal", PlannerGoal, self._goal_callback, queue_size=1)
        rospy.Subscriber(
            odom_topic,
            Odometry,
            self._odom_callback,
            queue_size=1,
            tcp_nodelay=True,
        )
        rospy.Subscriber(
            cloud_topic,
            PointCloud2,
            self._cloud_callback,
            queue_size=1,
            tcp_nodelay=True,
        )
        self.native_heartbeat_sub = rospy.Subscriber(
            native_heartbeat_topic,
            Empty,
            self._heartbeat_callback,
            queue_size=10,
            tcp_nodelay=True,
        )
        self.native_trajectory_sub = rospy.Subscriber(
            native_trajectory_topic,
            PolyTraj,
            self._trajectory_callback,
            queue_size=10,
            tcp_nodelay=True,
        )
        self.native_command_sub = rospy.Subscriber(
            native_command_topic,
            PositionCommand,
            self._command_callback,
            queue_size=20,
            tcp_nodelay=True,
        )
        rospy.Service("validate_goal", ValidateGoal, self._validate_goal)
        rospy.Timer(
            rospy.Duration(1.0 / self.status_rate), self._status_timer
        )
        rospy.Timer(rospy.Duration(1.0), self._capabilities_timer)
        self._publish_capabilities()
        self._publish_status()

    def _pose_type(self):
        from geometry_msgs.msg import PoseStamped

        return PoseStamped

    def _finite_param(self, name, default):
        value = float(self.rospy.get_param("~backend/" + name, default))
        if not math.isfinite(value):
            raise ValueError("~backend/{} must be finite".format(name))
        return value

    def _positive_param(self, name, default):
        value = self._finite_param(name, default)
        if value <= 0.0:
            raise ValueError("~backend/{} must be positive".format(name))
        return value

    def _goal_callback(self, message):
        reason = validate_pose_goal(
            message,
            self.virtual_ground,
            self.virtual_ceil,
            self.obstacle_inflation,
        )
        if reason:
            self.rospy.logerr("Rejected common goal: %s", reason)
            return
        if message.action == message.CANCEL:
            with self.lock:
                if self.session_id and message.session_id != self.session_id:
                    return
                should_stop = self.goal_id > 0
                self.cancelled = True
                self.native_armable = False
                self.reached_at = None
                self.planning_started_at = 0.0
                self.fault_reason = ""
            if should_stop:
                self._publish_recoverable_stop()
            self._publish_status()
            return

        public_goal = copy.deepcopy(message.goal)
        native_goal = copy.deepcopy(message.goal)
        native_stamp = self.rospy.Time.now()
        with self.lock:
            if (
                self.native_goal_stamp is not None
                and native_stamp <= self.native_goal_stamp
            ):
                native_stamp = self.native_goal_stamp + self.rospy.Duration(
                    nsecs=1
                )
            native_goal.header.stamp = native_stamp
            self.session_id = message.session_id
            self.goal_id = int(message.goal_id)
            self.goal = public_goal
            self.constrain_yaw = bool(message.constrain_yaw)
            self.native_goal_stamp = native_goal.header.stamp
            self.trajectory_id = 0
            self.native_armable = False
            self.cancelled = False
            self.reached_at = None
            self.planning_started_at = time.monotonic()
            self.fault_reason = ""
        # Publish only after all old command state has been invalidated.
        self.goal_pub.publish(native_goal)
        self._publish_status()

    def _odom_callback(self, message):
        now_ros = self.rospy.Time.now().to_sec()
        if (
            not valid_odometry(message)
            or not measurement_stamp_is_current(
                message.header.stamp,
                now_ros,
                self.odom_timeout,
                self.last_odom_stamp,
            )
        ):
            self.rospy.logwarn_throttle(
                1.0,
                "Ignoring invalid, stale, future, or replayed odometry.",
            )
            self._invalidate_sensor("odometry contract violation", odom=True)
            return
        with self.lock:
            self.odom = message
            self.last_odom_stamp = message.header.stamp.to_sec()
            self.last_odom_at = time.monotonic()
        self.native_odom_pub.publish(message)

    def _cloud_callback(self, message):
        now_ros = self.rospy.Time.now().to_sec()
        if (
            not valid_cloud_header(message)
            or not measurement_stamp_is_current(
                message.header.stamp,
                now_ros,
                self.cloud_timeout,
                self.last_cloud_stamp,
            )
        ):
            self.rospy.logwarn_throttle(
                1.0,
                "Ignoring invalid, stale, future, or replayed point cloud.",
            )
            self._invalidate_sensor(
                "point-cloud contract violation", cloud=True
            )
            return
        with self.lock:
            self.last_cloud_stamp = message.header.stamp.to_sec()
            self.last_cloud_at = time.monotonic()
        self.native_cloud_pub.publish(message)

    def _invalidate_sensor(self, reason, odom=False, cloud=False):
        with self.lock:
            if odom:
                self.last_odom_at = 0.0
                self.odom = None
            if cloud:
                self.last_cloud_at = 0.0
            should_stop = self.goal_id > 0 and not self.cancelled
            if self.goal_id > 0:
                self.native_armable = False
                self.cancelled = True
                self.reached_at = None
        if should_stop:
            self._publish_recoverable_stop()
        self.rospy.logerr_throttle(1.0, "Diff safety stop: %s", reason)
        self._publish_status()

    def _publish_recoverable_stop(self):
        stop = self.Header()
        with self.lock:
            stop.stamp = self.rospy.Time.now()
            if (
                self.native_goal_stamp is not None
                and stop.stamp < self.native_goal_stamp
            ):
                stop.stamp = self.native_goal_stamp
        stop.frame_id = WORLD_FRAME
        self.stop_pub.publish(stop)

    def _heartbeat_callback(self, _message):
        with self.lock:
            self.last_heartbeat_at = time.monotonic()

    def _trajectory_callback(self, message):
        if not valid_native_trajectory(message):
            self.rospy.logerr_throttle(
                1.0, "Ignoring invalid Diff PolyTraj id=%s.", message.traj_id
            )
            return
        with self.lock:
            if (
                self.native_goal_stamp is None
                or message.goal_stamp != self.native_goal_stamp
            ):
                return
            if int(message.traj_id) < self.trajectory_id:
                return
            self.trajectory_id = int(message.traj_id)
            self.native_armable = bool(message.armable)
            self.cancelled = not bool(message.armable)
            self.reached_at = None
            self.planning_started_at = 0.0
            self.fault_reason = ""
            if hasattr(self.goal, "pose") and finite(message.goal_position):
                self.goal.pose.position.x = float(message.goal_position[0])
                self.goal.pose.position.y = float(message.goal_position[1])
                self.goal.pose.position.z = float(message.goal_position[2])
        self._publish_status()

    def _command_callback(self, message):
        if not valid_native_command(message):
            self.rospy.logwarn_throttle(
                1.0,
                "Ignoring invalid Diff PositionCommand id=%s.",
                message.trajectory_id,
            )
            return
        with self.lock:
            if not self._runtime_ready_locked():
                return
            if (
                not self.session_id
                or self.goal_id <= 0
                or self.trajectory_id <= 0
                or int(message.trajectory_id) != self.trajectory_id
            ):
                return
            command = self.PlannerCommand()
            command.header = copy.deepcopy(message.header)
            if not valid_stamp(command.header.stamp):
                command.header.stamp = self.rospy.Time.now()
            command.header.frame_id = WORLD_FRAME
            command.session_id = self.session_id
            command.backend_id = self.backend_id
            command.goal_id = self.goal_id
            command.trajectory_id = self.trajectory_id
            reached = self._update_reached_locked()
            completed = (
                int(message.trajectory_flag)
                == int(message.TRAJECTORY_STATUS_COMPLETED)
            )
            if reached or completed:
                command.mode = command.HOLD
            elif self.native_armable and not self.cancelled:
                command.mode = command.NORMAL
            else:
                command.mode = command.BRAKE
            command.point = self._convert_point(message)
            if command.mode == command.HOLD:
                self._zero_point_motion(command.point)
        self.command_pub.publish(command)

    @staticmethod
    def _zero_point_motion(point):
        for twist in list(point.velocities) + list(point.accelerations):
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0

    def _convert_point(self, message):
        transform = self.Transform()
        transform.translation.x = message.position.x
        transform.translation.y = message.position.y
        transform.translation.z = message.position.z
        quaternion = self.quaternion_from_euler(0.0, 0.0, message.yaw)
        transform.rotation.x = quaternion[0]
        transform.rotation.y = quaternion[1]
        transform.rotation.z = quaternion[2]
        transform.rotation.w = quaternion[3]

        velocity = self.Twist()
        velocity.linear = copy.deepcopy(message.velocity)
        velocity.angular.z = message.yaw_dot
        acceleration = self.Twist()
        acceleration.linear = copy.deepcopy(message.acceleration)

        point = self.PlannerCommand().point
        point.transforms.append(transform)
        point.velocities.append(velocity)
        point.accelerations.append(acceleration)
        return point

    def _update_reached_locked(self):
        if self.goal is None or self.odom is None or not self.native_armable:
            self.reached_at = None
            return False
        current = self.odom.pose.pose.position
        target = self.goal.pose.position
        velocity = self.odom.twist.twist.linear
        error = math.sqrt(
            (current.x - target.x) ** 2
            + (current.y - target.y) ** 2
            + (current.z - target.z) ** 2
        )
        speed = math.sqrt(
            velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2
        )
        yaw_ready = True
        if self.constrain_yaw:
            current_yaw = quaternion_yaw(
                self.odom.pose.pose.orientation
            )
            target_yaw = quaternion_yaw(self.goal.pose.orientation)
            yaw_error = abs(wrapped_angle_error(target_yaw, current_yaw))
            yaw_rate = abs(float(self.odom.twist.twist.angular.z))
            yaw_ready = bool(
                yaw_error <= self.reached_yaw_tolerance
                and yaw_rate <= self.reached_yaw_rate_tolerance
            )
        now = time.monotonic()
        if (
            error <= self.goal_tolerance
            and speed <= self.reached_velocity_tolerance
            and yaw_ready
        ):
            if self.reached_at is None:
                self.reached_at = now
            return now - self.reached_at >= self.reached_hold_time
        self.reached_at = None
        return False

    def _readiness_locked(self):
        now = time.monotonic()
        return (
            self.last_odom_at > 0.0
            and now - self.last_odom_at <= self.odom_timeout,
            self.last_cloud_at > 0.0
            and now - self.last_cloud_at <= self.cloud_timeout,
            self.last_heartbeat_at > 0.0
            and now - self.last_heartbeat_at <= self.heartbeat_timeout,
            self.goal_pub.get_num_connections() > 0,
            self.native_trajectory_sub.get_num_connections() > 0
            and self.native_command_sub.get_num_connections() > 0,
        )

    def _runtime_ready_locked(self):
        return all(self._readiness_locked())

    def _status_locked(self):
        status = self.PlannerStatus()
        status.header.stamp = self.rospy.Time.now()
        status.header.frame_id = WORLD_FRAME
        status.session_id = self.session_id
        status.backend_id = self.backend_id
        status.goal_id = self.goal_id
        status.trajectory_id = self.trajectory_id
        (
            status.odom_ready,
            status.map_ready,
            heartbeat_ready,
            goal_link_ready,
            output_links_ready,
        ) = self._readiness_locked()
        healthy = (
            status.odom_ready
            and status.map_ready
            and heartbeat_ready
            and goal_link_ready
            and output_links_ready
        )
        status.armable = bool(
            healthy
            and self.native_armable
            and not self.cancelled
            and self.goal_id > 0
        )
        if hasattr(status, "active_goal") and self.goal is not None:
            status.active_goal = copy.deepcopy(self.goal)

        if not healthy:
            if not status.odom_ready:
                status.reason = "odometry unavailable, invalid, or stale"
            elif not status.map_ready:
                status.reason = "registered cloud unavailable, invalid, or stale"
            elif not goal_link_ready:
                status.reason = "Diff native goal interface is not connected"
            elif not output_links_ready:
                status.reason = "Diff native trajectory interfaces are not connected"
            else:
                status.reason = "Diff planner heartbeat unavailable or stale"
            if (
                self.ever_ready
                or time.monotonic() - self.started_at > self.startup_grace
            ):
                status.state = status.FAULT
            else:
                status.state = status.STARTING
            return status

        self.ever_ready = True
        if self.fault_reason:
            status.state = status.FAULT
            status.reason = self.fault_reason
        elif self.cancelled or (
            self.trajectory_id > 0 and not self.native_armable
        ):
            status.state = status.HOLDING
            status.reason = "goal cancelled or Diff emergency hold is active"
        elif self.goal_id <= 0:
            status.state = status.READY
        elif self.trajectory_id <= 0:
            status.state = status.PLANNING
        elif self._update_reached_locked():
            status.state = status.REACHED
        else:
            status.state = status.ACTIVE
        return status

    def _publish_status(self):
        with self.lock:
            status = self._status_locked()
        self.status_pub.publish(status)

    def _status_timer(self, _event):
        should_stop = False
        with self.lock:
            if (
                self.goal_id > 0
                and self.trajectory_id <= 0
                and not self.cancelled
                and self.planning_started_at > 0.0
                and time.monotonic() - self.planning_started_at
                > self.planning_timeout
            ):
                self.cancelled = True
                self.native_armable = False
                self.planning_started_at = 0.0
                self.fault_reason = (
                    "Diff planner did not produce a trajectory before "
                    "planning timeout"
                )
                should_stop = True
            status = self._status_locked()
        if should_stop:
            self._publish_recoverable_stop()
        self.status_pub.publish(status)

    def _publish_capabilities(self):
        message = self.PlannerCapabilities()
        message.header.stamp = self.rospy.Time.now()
        message.api_version = self.api_version
        message.backend_id = self.backend_id
        message.variant = "diff"
        message.simulation = True
        message.yaw = True
        message.cancel = True
        message.goal_validation = True
        message.rviz = True
        message.max_velocity = self.max_velocity
        message.max_acceleration = self.max_acceleration
        message.has_fixed_map_bounds = False
        self.capabilities_pub.publish(message)

    def _capabilities_timer(self, _event):
        self._publish_capabilities()

    def _validate_goal(self, request):
        reason = validate_pose_goal(
            request.goal,
            self.virtual_ground,
            self.virtual_ceil,
            self.obstacle_inflation,
        )
        return self.ValidateGoalResponse(valid=not reason, reason=reason)


def main():
    import rospy

    rospy.init_node("diff_backend_adapter")
    try:
        DiffBackendAdapter()
    except Exception as exc:
        rospy.logfatal("Diff backend adapter failed to initialize: %s", exc)
        raise
    rospy.spin()


if __name__ == "__main__":
    main()
