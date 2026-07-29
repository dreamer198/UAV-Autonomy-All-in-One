#!/usr/bin/env python3
"""Shared low-latency validated goal publisher for simulation and real flight."""

import argparse
import math
import signal
import socket
import sys
import threading
import time


EXIT_SUCCESS = 0
EXIT_FAILED = 1
LOCALIZATION_FAULT_PARAM = "/sim2real/localization_fault"


class GoalExecutorError(RuntimeError):
    pass


def goal_orientation(yaw_degrees):
    """Return the Planner quaternion fields for optional final yaw."""
    if yaw_degrees is None:
        # A zero-norm quaternion is the repository contract for unconstrained
        # final yaw; the trajectory server then keeps path-aligned yaw.
        return 0.0, 0.0
    if not math.isfinite(yaw_degrees):
        raise GoalExecutorError("goal yaw must be finite")
    yaw = math.radians(yaw_degrees)
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def validate_goal_coordinates(x, y, z):
    if not all(math.isfinite(value) for value in (x, y, z)):
        raise GoalExecutorError("goal coordinates must be finite")


def vertical_clearance_bounds(ground, ceil, inflation):
    values = (ground, ceil, inflation)
    if not all(math.isfinite(value) for value in values):
        raise GoalExecutorError("Planner vertical clearance values must be finite")
    if inflation < 0.0:
        raise GoalExecutorError("Planner obstacle inflation cannot be negative")
    minimum_z = ground + inflation
    maximum_z = ceil - inflation
    if minimum_z >= maximum_z:
        raise GoalExecutorError(
            "Planner vertical fence has no space after obstacle inflation"
        )
    return minimum_z, maximum_z


def localization_fault_reason(value):
    """Return the persistent interlock reason, or empty when it is clear."""
    if isinstance(value, dict):
        if not value.get("active", False):
            return ""
        return str(value.get("reason") or "localization safety fault")
    if value:
        return str(value)
    return ""


class SharedGoalExecutor:
    """Validate and publish one goal using one persistent set of ROS handles."""

    def __init__(self, rospy, args):
        import rosnode
        from geometry_msgs.msg import PoseStamped
        from mavros_msgs.msg import AttitudeTarget, State
        from nav_msgs.msg import Odometry
        from sim2real_planning_msgs.msg import PlannerGoal, PlannerStatus
        from sim2real_planning_msgs.srv import ValidateGoal

        self.rospy = rospy
        self.rosnode = rosnode
        self.PoseStamped = PoseStamped
        self.PlannerGoal = PlannerGoal
        self.PlannerStatus = PlannerStatus
        self.args = args
        self.condition = threading.Condition()
        self.abort_requested = False
        self.state = None
        self.state_received_at = 0.0
        self.odom_received_at = 0.0
        self.planner_status = None
        self.planner_status_received_at = 0.0
        self.attitude_setpoint_count = 0
        self.attitude_setpoint_received_at = 0.0
        self.started_at = time.monotonic()

        rospy.Subscriber(
            args.state_topic, State, self._state_callback, queue_size=1
        )
        rospy.Subscriber(
            args.odometry_topic,
            Odometry,
            self._odom_callback,
            queue_size=1,
            tcp_nodelay=True,
        )
        rospy.Subscriber(
            args.planner_status_topic,
            PlannerStatus,
            self._planner_status_callback,
            queue_size=1,
            tcp_nodelay=True,
        )
        if not args.allow_disarmed:
            rospy.Subscriber(
                args.attitude_setpoint_topic,
                AttitudeTarget,
                self._attitude_setpoint_callback,
                queue_size=max(20, args.attitude_setpoint_samples),
                tcp_nodelay=True,
            )
        self.goal_pub = rospy.Publisher(
            args.goal_topic, PoseStamped, queue_size=1, latch=False
        )
        self.validate_goal = rospy.ServiceProxy(
            args.validate_goal_service, ValidateGoal
        )

    def _state_callback(self, message):
        with self.condition:
            self.state = message
            self.state_received_at = time.monotonic()
            self.condition.notify_all()

    def _odom_callback(self, _message):
        with self.condition:
            self.odom_received_at = time.monotonic()
            self.condition.notify_all()

    def _planner_status_callback(self, message):
        with self.condition:
            self.planner_status = message
            self.planner_status_received_at = time.monotonic()
            self.condition.notify_all()

    def _attitude_setpoint_callback(self, _message):
        now = time.monotonic()
        with self.condition:
            if (
                self.attitude_setpoint_received_at > 0.0
                and now - self.attitude_setpoint_received_at
                > self.args.stream_gap_timeout
            ):
                self.attitude_setpoint_count = 0
            self.attitude_setpoint_count += 1
            self.attitude_setpoint_received_at = now
            self.condition.notify_all()

    def request_abort(self):
        with self.condition:
            self.abort_requested = True
            self.condition.notify_all()

    def _check_abort(self):
        if self.abort_requested or self.rospy.is_shutdown():
            raise GoalExecutorError("goal command was interrupted")

    def _check_localization_interlock(self):
        reason = localization_fault_reason(
            self.rospy.get_param(LOCALIZATION_FAULT_PARAM, "")
        )
        if reason:
            raise GoalExecutorError(
                "localization safety interlock is latched: {}. Restart the "
                "complete simulation/real stack before publishing a goal".format(
                    reason
                )
            )

    def _check_required_nodes(self):
        try:
            nodes = set(self.rosnode.get_node_names())
        except Exception as exc:
            raise GoalExecutorError("cannot query ROS nodes: {}".format(exc))

        if (
            not self.args.allow_disarmed
            and self.args.controller_node not in nodes
        ):
            raise GoalExecutorError(
                "controller node is not running: {}".format(
                    self.args.controller_node
                )
            )

    def _check_backend_goal(self):
        """Ask the selected backend to validate its own map and constraints."""
        qz, qw = goal_orientation(self.args.yaw_deg)
        pose = self.PoseStamped()
        pose.header.stamp = self.rospy.Time.now()
        pose.header.frame_id = self.args.frame_id
        pose.pose.position.x = self.args.x
        pose.pose.position.y = self.args.y
        pose.pose.position.z = self.args.z
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        goal = self.PlannerGoal()
        goal.header.stamp = pose.header.stamp
        goal.session_id = "preflight-validation"
        goal.goal_id = 1
        goal.action = goal.PLAN
        goal.goal = pose
        goal.constrain_yaw = self.args.yaw_deg is not None
        try:
            self.rospy.wait_for_service(
                self.args.validate_goal_service,
                timeout=self.args.preflight_timeout,
            )
            response = self.validate_goal(goal)
        except Exception as exc:
            raise GoalExecutorError(
                "selected planner goal validation failed: {}".format(exc)
            )
        if not response.valid:
            raise GoalExecutorError(
                "selected planner rejected the goal: {}".format(
                    response.reason or "unspecified validation failure"
                )
            )

    def _readiness_reason(self, now):
        self._check_localization_interlock()
        if self.state is None:
            return "waiting for MAVROS state"
        if now - self.state_received_at > self.args.state_timeout:
            return "waiting for fresh MAVROS state"
        if not self.state.connected:
            raise GoalExecutorError("MAVROS is not connected to PX4")
        if not self.args.allow_disarmed:
            if not self.state.armed or self.state.mode != "OFFBOARD":
                raise GoalExecutorError(
                    "vehicle must be armed in OFFBOARD (armed={}, mode={})".format(
                        self.state.armed, self.state.mode or "unknown"
                    )
                )
        if (
            self.odom_received_at <= 0.0
            or now - self.odom_received_at > self.args.odom_timeout
        ):
            return "waiting for fresh localization odometry"
        if (
            self.planner_status is None
            or now - self.planner_status_received_at
            > self.args.planner_status_timeout
        ):
            return "waiting for fresh selected-planner status"
        if self.planner_status.state == self.PlannerStatus.FAULT:
            raise GoalExecutorError(
                "selected planner is faulted: {}".format(
                    self.planner_status.reason or "unspecified fault"
                )
            )
        if not self.planner_status.odom_ready:
            return "waiting for selected planner odometry readiness"
        if not self.planner_status.map_ready:
            return "waiting for selected planner map readiness"
        if not self.args.allow_disarmed:
            if (
                self.attitude_setpoint_count
                < self.args.attitude_setpoint_samples
                or now - self.attitude_setpoint_received_at
                > self.args.stream_gap_timeout
            ):
                return "waiting for sustained SE3 attitude/thrust output"
        if self.goal_pub.get_num_connections() < self.args.goal_subscribers:
            return "waiting for Planner goal subscribers"
        return ""

    def _wait_for_readiness(self):
        deadline = time.monotonic() + self.args.preflight_timeout
        last_reason = "goal preflight has not started"
        with self.condition:
            while True:
                self._check_abort()
                now = time.monotonic()
                last_reason = self._readiness_reason(now)
                if not last_reason:
                    return
                remaining = deadline - now
                if remaining <= 0.0:
                    raise GoalExecutorError(
                        "{} was not ready within {:.1f}s".format(
                            last_reason, self.args.preflight_timeout
                        )
                    )
                self.condition.wait(min(remaining, 0.05))

    def _verify_ready_immediately_before_publish(self):
        with self.condition:
            reason = self._readiness_reason(time.monotonic())
        if reason:
            raise GoalExecutorError(
                "goal readiness changed before publication: {}".format(reason)
            )

    def run(self):
        try:
            self._check_localization_interlock()
            self._check_required_nodes()
            self._wait_for_readiness()
            self._check_backend_goal()
            self._verify_ready_immediately_before_publish()

            qz, qw = goal_orientation(self.args.yaw_deg)
            message = self.PoseStamped()
            message.header.stamp = self.rospy.Time.now()
            message.header.frame_id = self.args.frame_id
            message.pose.position.x = self.args.x
            message.pose.position.y = self.args.y
            message.pose.position.z = self.args.z
            message.pose.orientation.z = qz
            message.pose.orientation.w = qw
            self.goal_pub.publish(message)

            # Connections are already established. Keep the process alive for
            # one short transport interval so the non-latched message is put on
            # both subscriber sockets before rospy unregisters the publisher.
            time.sleep(self.args.delivery_wait)
            yaw_description = (
                "unconstrained"
                if self.args.yaw_deg is None
                else "{:.3f} deg".format(self.args.yaw_deg)
            )
            self.rospy.loginfo(
                "Published goal (%.3f, %.3f, %.3f), yaw=%s after %.2f s; "
                "subscribers=%d.",
                self.args.x,
                self.args.y,
                self.args.z,
                yaw_description,
                time.monotonic() - self.started_at,
                self.goal_pub.get_num_connections(),
            )
            return EXIT_SUCCESS
        except GoalExecutorError as exc:
            self.rospy.logerr("Shared goal executor refused the goal: %s", exc)
            return EXIT_FAILED
        except Exception as exc:
            self.rospy.logerr(
                "Shared goal executor refused the goal after an unexpected "
                "runtime error: %s",
                exc,
            )
            return EXIT_FAILED


def _build_parser():
    parser = argparse.ArgumentParser(
        description="Validate and publish one shared simulation/real goal"
    )
    parser.add_argument("x", type=float)
    parser.add_argument("y", type=float)
    parser.add_argument("z", type=float)
    parser.add_argument("--yaw-deg", type=float)
    parser.add_argument("--drone-id", type=int, default=0)
    parser.add_argument("--frame-id", default="world")
    parser.add_argument("--goal-topic", default="/goal")
    parser.add_argument("--state-topic", default="/mavros/state")
    parser.add_argument("--odometry-topic", default="/localization/odom")
    parser.add_argument("--planner-status-topic", default="/planning/status")
    parser.add_argument(
        "--validate-goal-service", default="/planning/validate_goal"
    )
    parser.add_argument(
        "--attitude-setpoint-topic",
        default="/mavros/setpoint_raw/attitude",
    )
    parser.add_argument("--controller-node", default="/se3_controller_node")
    parser.add_argument("--preflight-timeout", type=float, default=5.0)
    parser.add_argument("--state-timeout", type=float, default=3.0)
    parser.add_argument("--odom-timeout", type=float, default=0.5)
    parser.add_argument("--stream-gap-timeout", type=float, default=0.5)
    parser.add_argument("--attitude-setpoint-samples", type=int, default=10)
    parser.add_argument("--goal-subscribers", type=int, default=1)
    parser.add_argument("--planner-status-timeout", type=float, default=0.5)
    parser.add_argument("--delivery-wait", type=float, default=0.05)
    parser.add_argument(
        "--allow-disarmed",
        action="store_true",
        help="planner-only test mode; still requires connected PX4 and odometry",
    )
    return parser


def _validate_args(parser, args):
    try:
        validate_goal_coordinates(args.x, args.y, args.z)
        goal_orientation(args.yaw_deg)
    except GoalExecutorError as exc:
        parser.error(str(exc))
    for name in (
        "preflight_timeout",
        "state_timeout",
        "odom_timeout",
        "stream_gap_timeout",
        "planner_status_timeout",
        "delivery_wait",
    ):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0.0:
            parser.error(
                "--{} must be finite and positive".format(name.replace("_", "-"))
            )
    if args.drone_id < 0:
        parser.error("--drone-id must be non-negative")
    if args.attitude_setpoint_samples <= 0:
        parser.error("--attitude-setpoint-samples must be positive")
    if args.goal_subscribers <= 0:
        parser.error("--goal-subscribers must be positive")
    if not args.frame_id:
        parser.error("--frame-id cannot be empty")


def main(argv=None):
    parser = _build_parser()
    args = parser.parse_args(argv)
    _validate_args(parser, args)

    try:
        import rosgraph
        import rospy

        previous_socket_timeout = socket.getdefaulttimeout()
        socket.setdefaulttimeout(args.preflight_timeout)
        try:
            rosgraph.Master("/shared_goal_executor_probe").getPid()
        finally:
            socket.setdefaulttimeout(previous_socket_timeout)
        rospy.init_node("shared_goal_executor", disable_signals=True)
        executor = SharedGoalExecutor(rospy, args)
    except Exception as exc:  # ROS master/API exceptions vary by ROS release
        print(
            "[ERROR] Cannot initialize shared goal executor: {}".format(exc),
            file=sys.stderr,
        )
        return EXIT_FAILED

    previous_sigint = signal.getsignal(signal.SIGINT)
    previous_sigterm = signal.getsignal(signal.SIGTERM)

    def request_abort(_signum, _frame):
        executor.request_abort()

    signal.signal(signal.SIGINT, request_abort)
    signal.signal(signal.SIGTERM, request_abort)
    try:
        return executor.run()
    finally:
        signal.signal(signal.SIGINT, previous_sigint)
        signal.signal(signal.SIGTERM, previous_sigterm)
        rospy.signal_shutdown("shared goal command finished")


if __name__ == "__main__":
    sys.exit(main())
