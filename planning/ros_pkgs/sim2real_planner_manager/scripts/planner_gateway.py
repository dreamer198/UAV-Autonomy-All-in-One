#!/usr/bin/env python3
"""Safety gateway between a selected planner plugin and the SE3 controller."""

import copy
from collections import deque
import math
import os
import threading
import time
import uuid

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from std_srvs.srv import Trigger, TriggerResponse
from trajectory_msgs.msg import MultiDOFJointTrajectory

from sim2real_planning_msgs.msg import (
    PlannerCapabilities,
    PlannerCommand,
    PlannerGoal,
    PlannerStatus,
)
from sim2real_planning_msgs.srv import (
    ValidateGoal,
    ValidateGoalResponse,
)
from sim2real_planner_manager.command_gate import (
    CommandGate,
    GateConfig,
    StatusSnapshot,
)
from sim2real_planner_manager.manifest import (
    API_VERSION,
    ManifestError,
    discover_plugins,
)
from sim2real_planner_manager.validation import (
    backend_status_allows_new_goal,
    validate_command_mode,
    validate_fixed_bounds,
    validate_goal_pose,
    status_order_is_newer,
    validate_trajectory_point,
)


class PlannerGateway:
    def __init__(self):
        self._lock = threading.RLock()
        self._planner_id = rospy.get_param("~planner_id", "diff")
        self._runtime_mode = rospy.get_param("~runtime_mode", "")
        manifest_root = rospy.get_param("~manifest_root", "")
        repository_root = rospy.get_param("~repository_root", "")
        try:
            plugins = discover_plugins(
                builtin_root=manifest_root or None,
                require_runtime=False,
                repository_root=repository_root or None,
            )
            self._manifest = plugins[self._planner_id]
        except KeyError:
            raise RuntimeError(
                "unknown planner {!r}; discovered: {}".format(
                    self._planner_id, ", ".join(sorted(plugins))
                )
            )
        except ManifestError as exc:
            raise RuntimeError("planner manifest validation failed: {}".format(exc))
        environment_mode = os.environ.get("SIM2REAL_RUNTIME_MODE", "")
        if self._runtime_mode not in {"simulation", "real"}:
            raise RuntimeError(
                "runtime_mode must be explicitly set to simulation or real"
            )
        if environment_mode not in {"simulation", "real"}:
            raise RuntimeError(
                "SIM2REAL_RUNTIME_MODE must be explicitly set to simulation or real"
            )
        if environment_mode != self._runtime_mode:
            raise RuntimeError(
                "runtime_mode {!r} disagrees with container mode {!r}".format(
                    self._runtime_mode, environment_mode
                )
            )
        if not self._manifest.supports_runtime(self._runtime_mode):
            raise RuntimeError(
                "planner {!r} is not enabled for {} runtime".format(
                    self._planner_id, self._runtime_mode
                )
            )

        self._session_id = rospy.get_param("~session_id", "") or uuid.uuid4().hex
        self._goal_frame = rospy.get_param("~goal_frame", "world")
        self._require_offboard = bool(
            rospy.get_param("~require_offboard", True)
        )
        self._accept_goals_without_offboard = bool(
            rospy.get_param("~accept_goals_without_offboard", False)
        )
        self._status_timeout = float(
            rospy.get_param(
                "~status_timeout", self._manifest.timeouts.status_sec
            )
        )
        self._command_timeout = float(
            rospy.get_param(
                "~command_timeout", self._manifest.timeouts.command_sec
            )
        )
        self._startup_timeout = float(
            rospy.get_param(
                "~startup_timeout", self._manifest.timeouts.startup_sec
            )
        )
        self._rate_window = float(rospy.get_param("~rate_window", 1.0))
        if (
            not math.isfinite(self._startup_timeout)
            or self._startup_timeout <= 0.0
            or not math.isfinite(self._rate_window)
            or self._rate_window <= 0.0
        ):
            raise RuntimeError("startup_timeout and rate_window must be positive")
        self._state_timeout = float(rospy.get_param("~state_timeout", 3.0))
        self._backend_service_timeout = float(
            rospy.get_param("~backend_service_timeout", 2.0)
        )
        self._enforce_unique_output = bool(
            rospy.get_param("~enforce_unique_output", True)
        )
        self._gate = CommandGate(
            GateConfig(
                backend_id=self._planner_id,
                session_id=self._session_id,
                require_offboard=self._require_offboard,
                state_timeout_sec=self._state_timeout,
                status_timeout_sec=self._status_timeout,
                command_timeout_sec=self._command_timeout,
            )
        )

        backend = self._manifest.backend_namespace
        self._backend_caller_id = (
            backend + "/" + self._manifest.adapter_node
        )
        self._backend_goal_pub = rospy.Publisher(
            backend + "/goal", PlannerGoal, queue_size=2, latch=False
        )
        self._assigned_goal_pub = rospy.Publisher(
            "/planning/goal", PlannerGoal, queue_size=2, latch=False
        )
        self._accepted_command_pub = rospy.Publisher(
            "/planning/command", PlannerCommand, queue_size=1
        )
        self._status_pub = rospy.Publisher(
            "/planning/status", PlannerStatus, queue_size=10, latch=True
        )
        self._capabilities_pub = rospy.Publisher(
            "/planning/capabilities",
            PlannerCapabilities,
            queue_size=1,
            latch=True,
        )
        self._output_topic = rospy.resolve_name(
            rospy.get_param("~output_topic", "/command/trajectory")
        )
        self._output_pub = rospy.Publisher(
            self._output_topic,
            MultiDOFJointTrajectory,
            queue_size=1,
        )

        self._backend_validate_name = backend + "/validate_goal"
        self._backend_validate = rospy.ServiceProxy(
            self._backend_validate_name, ValidateGoal, persistent=False
        )
        self._goal_id = 0
        self._goal_request_generation = 0
        self._started_at = self._now_monotonic()
        self._backend_ready_since = 0.0
        self._ever_backend_ready = False
        self._status_receipts = deque(maxlen=4096)
        self._command_receipts = deque(maxlen=4096)
        self._goal_started_at = 0.0
        self._last_goal = None
        self._last_backend_status = None
        self._last_backend_status_at = 0.0
        self._last_backend_status_stamp = rospy.Time()
        self._last_backend_status_seq = -1
        self._capabilities = None
        self._capabilities_valid = False
        self._fault_reported = False

        rospy.Subscriber(
            rospy.get_param("~goal_topic", "/goal"),
            PoseStamped,
            self._goal_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            rospy.get_param("~mavros_state_topic", "/mavros/state"),
            State,
            self._state_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            backend + "/command",
            PlannerCommand,
            self._command_callback,
            queue_size=1,
            tcp_nodelay=True,
        )
        rospy.Subscriber(
            backend + "/status",
            PlannerStatus,
            self._status_callback,
            queue_size=10,
            tcp_nodelay=True,
        )
        rospy.Subscriber(
            backend + "/capabilities",
            PlannerCapabilities,
            self._capabilities_callback,
            queue_size=1,
        )
        rospy.Service(
            "/planning/validate_goal", ValidateGoal, self._validate_goal_service
        )
        rospy.Service("/planning/cancel", Trigger, self._cancel_service)
        self._watchdog_timer = rospy.Timer(
            rospy.Duration(min(0.02, self._command_timeout / 4.0)),
            self._watchdog_callback,
        )
        self._publisher_timer = rospy.Timer(
            rospy.Duration(2.0), self._publisher_check_callback
        )
        self._publish_lifecycle(
            PlannerStatus.STARTING, "waiting for selected planner backend"
        )
        rospy.loginfo(
            "[planner_gateway] session=%s backend=%s namespace=%s",
            self._session_id,
            self._planner_id,
            backend,
        )

    @staticmethod
    def _now_monotonic():
        return time.monotonic()

    def _publish_lifecycle(self, state, reason):
        status = PlannerStatus()
        status.header.stamp = rospy.Time.now()
        status.header.frame_id = self._goal_frame
        status.session_id = self._session_id
        status.backend_id = self._planner_id
        status.goal_id = self._goal_id
        status.trajectory_id = 0
        status.state = state
        if self._last_goal is not None:
            status.active_goal = copy.deepcopy(self._last_goal)
        if self._last_backend_status is not None:
            status.odom_ready = self._last_backend_status.odom_ready
            status.map_ready = self._last_backend_status.map_ready
        status.armable = False
        status.reason = reason
        self._status_pub.publish(status)

    def _backend_runtime_ready(self):
        return bool(
            self._capabilities_valid
            and self._backend_goal_pub.get_num_connections() > 0
            and self._last_backend_status is not None
            and self._last_backend_status.state == PlannerStatus.READY
            and self._last_backend_status.odom_ready
            and self._last_backend_status.map_ready
        )

    def _backend_accepts_goal(self, now):
        return bool(
            self._capabilities_valid
            and self._backend_goal_pub.get_num_connections() > 0
            and backend_status_allows_new_goal(
                self._last_backend_status,
                received_at=self._last_backend_status_at,
                now=now,
                timeout=self._status_timeout,
                allowed_states={
                    PlannerStatus.READY,
                    PlannerStatus.PLANNING,
                    PlannerStatus.ACTIVE,
                    PlannerStatus.HOLDING,
                    PlannerStatus.REACHED,
                    PlannerStatus.FAULT,
                },
            )
        )

    def _trusted_backend_message(self, message, message_kind):
        connection_header = getattr(message, "_connection_header", None) or {}
        caller_id = connection_header.get("callerid", "")
        if caller_id != self._backend_caller_id:
            rospy.logerr_throttle(
                1.0,
                "[planner_gateway] rejected %s from untrusted caller %r; expected %r",
                message_kind,
                caller_id,
                self._backend_caller_id,
            )
            return False
        return True

    def _update_ready_time(self, now):
        if self._backend_runtime_ready():
            self._ever_backend_ready = True
            if self._backend_ready_since <= 0.0:
                self._backend_ready_since = now
        else:
            self._backend_ready_since = 0.0

    def _observed_rate(self, receipts, now):
        if not receipts or now - receipts[0] < self._rate_window:
            return None
        cutoff = now - self._rate_window
        recent = [stamp for stamp in receipts if stamp >= cutoff]
        if len(recent) < 2:
            return 0.0
        elapsed = recent[-1] - recent[0]
        if elapsed <= 0.0:
            return 0.0
        return float(len(recent) - 1) / elapsed

    def _preopen_command_rate_ready(self):
        minimum_rate = self._manifest.rates.command_min_hz
        sample_count = max(3, int(math.ceil(minimum_rate * 0.1)) + 1)
        if len(self._command_receipts) < sample_count:
            return False
        samples = list(self._command_receipts)[-sample_count:]
        elapsed = samples[-1] - samples[0]
        return (
            elapsed > 0.0
            and float(sample_count - 1) / elapsed >= minimum_rate
        )

    def _state_callback(self, message):
        now = self._now_monotonic()
        with self._lock:
            was_open = self._gate.is_open
            was_revoked = self._gate.is_revoked
            self._gate.update_vehicle(
                connected=message.connected,
                armed=message.armed,
                mode=message.mode,
                received_at=now,
            )
            if was_open and not self._gate.is_open:
                rospy.logwarn(
                    "[planner_gateway] command gate closed after leaving armed OFFBOARD"
                )
            if (
                self._goal_id > 0
                and not was_revoked
                and self._gate.is_revoked
            ):
                self._fault_reported = True
                self._publish_lifecycle(
                    PlannerStatus.FAULT,
                    "vehicle left connected armed OFFBOARD; publish a new goal",
                )

    def _normalize_goal(self, message):
        valid, reason, constrain_yaw = validate_goal_pose(
            message, expected_frame=self._goal_frame
        )
        if not valid:
            return None, reason
        if constrain_yaw and not self._manifest.capabilities.yaw:
            return None, "selected backend does not support constrained yaw"
        normalized = copy.deepcopy(message)
        if constrain_yaw:
            q = normalized.pose.orientation
            norm = math.sqrt(
                q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w
            )
            q.x /= norm
            q.y /= norm
            q.z /= norm
            q.w /= norm
        request = PlannerGoal()
        request.header.stamp = rospy.Time.now()
        request.header.frame_id = self._goal_frame
        request.session_id = self._session_id
        request.goal_id = self._goal_id + 1
        request.action = PlannerGoal.PLAN
        request.goal = normalized
        request.constrain_yaw = constrain_yaw
        return request, ""

    def _goal_callback(self, message):
        now = self._now_monotonic()
        with self._lock:
            if not self._capabilities_valid:
                rospy.logerr_throttle(
                    1.0,
                    "[planner_gateway] rejecting goal until backend capabilities are valid",
                )
                return
            if not self._backend_accepts_goal(now):
                rospy.logwarn_throttle(
                    1.0,
                    "[planner_gateway] rejecting goal until backend is healthy",
                )
                return
            if (
                self._require_offboard
                and not self._accept_goals_without_offboard
                and not self._gate.vehicle_ready(now)
            ):
                rospy.logwarn_throttle(
                    1.0,
                    "[planner_gateway] rejecting goal before connected armed OFFBOARD",
                )
                return
            request, reason = self._normalize_goal(message)
            if request is None:
                rospy.logerr("[planner_gateway] rejecting invalid goal: %s", reason)
                return
            self._goal_request_generation += 1
            generation = self._goal_request_generation

        if self._manifest.capabilities.goal_validation:
            try:
                rospy.wait_for_service(
                    self._backend_validate_name,
                    timeout=self._backend_service_timeout,
                )
                validation = self._backend_validate(request)
            except (rospy.ROSException, rospy.ServiceException) as exc:
                rospy.logerr(
                    "[planner_gateway] rejecting goal because backend validation "
                    "is unavailable: %s",
                    exc,
                )
                return
            if not validation.valid:
                rospy.logerr(
                    "[planner_gateway] backend rejected goal: %s",
                    validation.reason,
                )
                return

        now = self._now_monotonic()
        with self._lock:
            if generation != self._goal_request_generation:
                rospy.logwarn(
                    "[planner_gateway] dropping superseded goal validation result"
                )
                return
            if (
                not self._backend_accepts_goal(now)
                or request.goal_id != self._goal_id + 1
            ):
                rospy.logwarn(
                    "[planner_gateway] backend became unhealthy while validating goal"
                )
                return
            if (
                self._require_offboard
                and not self._accept_goals_without_offboard
                and not self._gate.vehicle_ready(now)
            ):
                rospy.logwarn(
                    "[planner_gateway] vehicle left armed OFFBOARD while validating goal"
                )
                return
            self._goal_id = request.goal_id
            self._goal_started_at = now
            self._last_goal = copy.deepcopy(request.goal)
            # Closing the command gate precedes publishing any evidence of the
            # new goal, so a racing old command can never pass through.
            self._gate.begin_goal(self._goal_id)
            self._command_receipts.clear()
            self._fault_reported = False
            self._publish_lifecycle(
                PlannerStatus.PLANNING, "goal accepted; waiting for backend plan"
            )
            self._assigned_goal_pub.publish(request)
            self._backend_goal_pub.publish(request)

    def _status_callback(self, message):
        now = self._now_monotonic()
        if not self._trusted_backend_message(message, "status"):
            return
        stamp_valid = not message.header.stamp.is_zero()
        stamp_age = (
            (rospy.Time.now() - message.header.stamp).to_sec()
            if stamp_valid
            else float("inf")
        )
        with self._lock:
            if (
                message.header.frame_id != self._goal_frame
                or
                not stamp_valid
                or not math.isfinite(stamp_age)
                or stamp_age < -self._status_timeout
                or stamp_age > self._status_timeout
                or (
                    not self._last_backend_status_stamp.is_zero()
                    and not status_order_is_newer(
                        message.header.stamp.to_sec(),
                        int(message.header.seq),
                        self._last_backend_status_stamp.to_sec(),
                        self._last_backend_status_seq,
                    )
                )
            ):
                rospy.logwarn_throttle(
                    1.0,
                    "[planner_gateway] rejected stale, future, or replayed backend status",
                )
                return
            if message.backend_id != self._planner_id:
                rospy.logerr_throttle(
                    1.0, "[planner_gateway] rejected status from wrong backend"
                )
                return
            if self._goal_id == 0:
                if (
                    message.goal_id != 0
                    or message.state
                    not in {
                        PlannerStatus.STARTING,
                        PlannerStatus.READY,
                        PlannerStatus.FAULT,
                    }
                    or message.session_id not in {"", self._session_id}
                ):
                    rospy.logwarn_throttle(
                        1.0,
                        "[planner_gateway] rejected invalid pre-goal backend status",
                    )
                    return
                accepted = copy.deepcopy(message)
                accepted.session_id = self._session_id
            else:
                was_revoked = self._gate.is_revoked
                decision = self._gate.update_status(
                    StatusSnapshot(
                        session_id=message.session_id,
                        backend_id=message.backend_id,
                        goal_id=message.goal_id,
                        trajectory_id=message.trajectory_id,
                        state=message.state,
                        odom_ready=message.odom_ready,
                        map_ready=message.map_ready,
                        armable=message.armable,
                        received_at=now,
                    )
                )
                if not decision.accepted:
                    rospy.logwarn_throttle(
                        1.0,
                        "[planner_gateway] rejected backend status: %s",
                        decision.reason,
                    )
                    return
                accepted = copy.deepcopy(message)
            self._last_backend_status = accepted
            self._last_backend_status_at = now
            self._last_backend_status_stamp = message.header.stamp
            self._last_backend_status_seq = int(message.header.seq)
            self._status_receipts.append(now)
            self._fault_reported = (
                self._goal_id > 0 and self._gate.is_revoked
            )
            if self._goal_id > 0 and was_revoked:
                rospy.logwarn_throttle(
                    1.0,
                    "[planner_gateway] retaining backend health only after goal "
                    "authorization was revoked",
                )
                self._update_ready_time(now)
                if accepted.odom_ready and accepted.map_ready:
                    recoverable = copy.deepcopy(accepted)
                    recoverable.state = PlannerStatus.HOLDING
                    recoverable.armable = False
                    recoverable.reason = (
                        "goal authorization is revoked; submit a new goal"
                    )
                    self._status_pub.publish(recoverable)
                return
            if (
                accepted.state == PlannerStatus.READY
                and not self._capabilities_valid
            ):
                self._publish_lifecycle(
                    PlannerStatus.STARTING,
                    "backend ready; waiting for validated capabilities",
                )
            else:
                self._status_pub.publish(accepted)
            self._update_ready_time(now)

    def _capabilities_callback(self, message):
        if not self._trusted_backend_message(message, "capabilities"):
            return
        expected = self._manifest.capabilities
        mismatch = []
        pairs = {
            "simulation": expected.simulation,
            "real_flight": expected.real_flight,
            "yaw": expected.yaw,
            "cancel": expected.cancel,
            "goal_validation": expected.goal_validation,
            "rviz": expected.rviz,
        }
        if message.api_version != API_VERSION:
            mismatch.append("api_version")
        if message.backend_id != self._planner_id:
            mismatch.append("backend_id")
        if message.variant != self._manifest.variant:
            mismatch.append("variant")
        for field, declared in pairs.items():
            if getattr(message, field) != declared:
                mismatch.append(field)
        numeric_valid, numeric_reason = validate_fixed_bounds(message)
        if not numeric_valid:
            mismatch.append(numeric_reason)
        with self._lock:
            self._capabilities_valid = not mismatch
            if mismatch:
                self._capabilities = None
                self._gate.force_close()
                self._publish_lifecycle(
                    PlannerStatus.FAULT,
                    "runtime capabilities disagree with manifest: {}".format(
                        ", ".join(mismatch)
                    ),
                )
                rospy.logerr(
                    "[planner_gateway] invalid runtime capabilities: %s",
                    ", ".join(mismatch),
                )
                return
            accepted = copy.deepcopy(message)
            accepted.header.stamp = rospy.Time.now()
            self._capabilities = copy.deepcopy(accepted)
            self._capabilities_pub.publish(accepted)
            if self._goal_id == 0 and self._last_backend_status is not None:
                ready = copy.deepcopy(self._last_backend_status)
                ready.session_id = self._session_id
                self._status_pub.publish(ready)
            self._update_ready_time(self._now_monotonic())

    def _command_callback(self, message):
        if not self._trusted_backend_message(message, "command"):
            return
        shape_valid, shape_reason = validate_trajectory_point(message.point)
        if shape_valid and message.header.frame_id != self._goal_frame:
            shape_valid = False
            shape_reason = "command frame must be {!r}".format(
                self._goal_frame
            )
        if shape_valid:
            shape_valid, shape_reason = validate_command_mode(
                message.mode, message.point, PlannerCommand.HOLD
            )
        with self._lock:
            now = self._now_monotonic()
            stamp_valid = not message.header.stamp.is_zero()
            header_age = (
                (rospy.Time.now() - message.header.stamp).to_sec()
                if stamp_valid
                else float("inf")
            )
            if not self._capabilities_valid:
                return
            receipt_recorded = False
            status = self._gate.status
            preopen_candidate = bool(
                not self._gate.is_open
                and not self._gate.is_revoked
                and message.mode == PlannerCommand.NORMAL
                and message.session_id == self._session_id
                and message.backend_id == self._planner_id
                and message.goal_id == self._goal_id
                and message.trajectory_id > 0
                and status is not None
                and status.goal_id == message.goal_id
                and status.trajectory_id == message.trajectory_id
                and status.state == PlannerStatus.ACTIVE
                and status.armable
                and status.odom_ready
                and status.map_ready
                and shape_valid
                and stamp_valid
                and math.isfinite(header_age)
                and abs(header_age) <= self._command_timeout
                and self._gate.vehicle_ready(now)
            )
            if preopen_candidate:
                self._command_receipts.append(now)
                receipt_recorded = True
                if not self._preopen_command_rate_ready():
                    rospy.loginfo_throttle(
                        1.0,
                        "[planner_gateway] warming up backend command-rate gate",
                    )
                    return
            was_revoked = self._gate.is_revoked
            decision = self._gate.evaluate_command(
                session_id=message.session_id,
                backend_id=message.backend_id,
                goal_id=message.goal_id,
                trajectory_id=message.trajectory_id,
                mode=message.mode,
                values_finite=shape_valid,
                shape_valid=shape_valid and stamp_valid,
                received_at=now,
                header_age_sec=header_age,
            )
            if not decision.accepted:
                if not was_revoked and self._gate.is_revoked:
                    self._fault_reported = True
                    self._publish_lifecycle(
                        PlannerStatus.FAULT,
                        "command gate revoked: {}".format(decision.reason),
                    )
                rospy.logwarn_throttle(
                    1.0,
                    "[planner_gateway] rejected command: %s%s",
                    decision.reason,
                    " ({})".format(shape_reason) if shape_reason else "",
                )
                return
            output = MultiDOFJointTrajectory()
            output.header = copy.deepcopy(message.header)
            output.points.append(copy.deepcopy(message.point))
            self._accepted_command_pub.publish(message)
            self._output_pub.publish(output)
            if not receipt_recorded:
                self._command_receipts.append(now)
            if decision.opened_gate:
                rospy.loginfo(
                    "[planner_gateway] opened command gate for goal=%d trajectory=%d",
                    message.goal_id,
                    message.trajectory_id,
                )

    def _normalized_validation_request(self, request):
        if request.goal.action != PlannerGoal.PLAN:
            return None, "only PLAN requests can be validated"
        valid, reason, constrain_yaw = validate_goal_pose(
            request.goal.goal, expected_frame=self._goal_frame
        )
        if not valid:
            return None, reason
        if constrain_yaw and not self._manifest.capabilities.yaw:
            return None, "selected backend does not support constrained yaw"
        normalized = copy.deepcopy(request)
        normalized.goal.session_id = self._session_id
        normalized.goal.goal_id = max(self._goal_id + 1, 1)
        normalized.goal.constrain_yaw = constrain_yaw
        return normalized, ""

    def _validate_goal_service(self, request):
        with self._lock:
            if not self._capabilities_valid:
                return ValidateGoalResponse(
                    valid=False, reason="backend capabilities are not ready"
                )
            if not self._manifest.capabilities.goal_validation:
                return ValidateGoalResponse(
                    valid=False, reason="selected backend does not validate goals"
                )
            normalized, reason = self._normalized_validation_request(request)
            if normalized is None:
                return ValidateGoalResponse(valid=False, reason=reason)
        try:
            rospy.wait_for_service(
                self._backend_validate_name,
                timeout=self._backend_service_timeout,
            )
            return self._backend_validate(normalized.goal)
        except (rospy.ROSException, rospy.ServiceException) as exc:
            return ValidateGoalResponse(
                valid=False,
                reason="backend goal validation unavailable: {}".format(exc),
            )

    def _cancel_service(self, _request):
        with self._lock:
            if self._goal_id <= 0:
                return TriggerResponse(success=False, message="no active goal")
            if not self._manifest.capabilities.cancel:
                return TriggerResponse(
                    success=False,
                    message="selected backend does not support cancellation",
                )
            cancel = PlannerGoal()
            cancel.header.stamp = rospy.Time.now()
            cancel.header.frame_id = self._goal_frame
            cancel.session_id = self._session_id
            cancel.goal_id = self._goal_id
            cancel.action = PlannerGoal.CANCEL
            if self._last_goal is not None:
                cancel.goal = copy.deepcopy(self._last_goal)
            self._gate.cancel_goal()
            self._goal_request_generation += 1
            self._command_receipts.clear()
            self._goal_started_at = 0.0
            self._fault_reported = False
            self._assigned_goal_pub.publish(cancel)
            self._backend_goal_pub.publish(cancel)
            self._publish_lifecycle(PlannerStatus.READY, "goal cancelled")
            return TriggerResponse(success=True, message="goal cancelled")

    def _watchdog_callback(self, _event):
        now = self._now_monotonic()
        with self._lock:
            if self._goal_id <= 0:
                backend_status_stale = (
                    self._last_backend_status_at > 0.0
                    and now - self._last_backend_status_at
                    > self._status_timeout
                )
                if (
                    backend_status_stale
                    or (
                        not self._backend_runtime_ready()
                        and not self._ever_backend_ready
                        and now - self._started_at > self._startup_timeout
                    )
                ):
                    self._gate.force_close()
                    self._publish_lifecycle(
                        PlannerStatus.FAULT,
                        (
                            "selected backend status timed out"
                            if backend_status_stale
                            else "selected backend did not become READY before startup timeout"
                        ),
                    )
                    rospy.signal_shutdown(
                        "planner backend status timeout"
                        if backend_status_stale
                        else "planner backend startup timeout"
                    )
                    return
                status_rate = self._observed_rate(
                    self._status_receipts, now
                )
                if (
                    self._backend_ready_since > 0.0
                    and now - self._backend_ready_since >= self._rate_window
                    and status_rate is not None
                    and status_rate < self._manifest.rates.status_min_hz
                ):
                    self._gate.force_close()
                    self._publish_lifecycle(
                        PlannerStatus.FAULT,
                        "selected backend status rate is below manifest minimum",
                    )
                    rospy.signal_shutdown("planner backend status rate too low")
                return
            if self._fault_reported:
                return
            status = self._gate.status
            status_timed_out = (
                status is not None
                and now - status.received_at > self._status_timeout
            )
            initial_status_timed_out = (
                status is None
                and self._goal_started_at > 0.0
                and now - self._goal_started_at > self._status_timeout
            )
            command_timed_out = (
                self._gate.is_open
                and self._gate.last_command_at is not None
                and now - self._gate.last_command_at > self._command_timeout
            )
            vehicle_timed_out = (
                self._require_offboard
                and not self._gate.vehicle_ready(now)
            )
            status_rate = self._observed_rate(self._status_receipts, now)
            command_rate = self._observed_rate(self._command_receipts, now)
            status_rate_low = (
                status_rate is not None
                and status_rate < self._manifest.rates.status_min_hz
            )
            command_rate_low = (
                self._gate.is_open
                and command_rate is not None
                and command_rate < self._manifest.rates.command_min_hz
            )
            if (
                status_timed_out
                or initial_status_timed_out
                or command_timed_out
                or vehicle_timed_out
                or status_rate_low
                or command_rate_low
            ):
                self._gate.force_close()
                self._fault_reported = True
                if command_rate_low:
                    reason = "selected backend command rate is below manifest minimum"
                elif status_rate_low:
                    reason = "selected backend status rate is below manifest minimum"
                elif command_timed_out:
                    reason = "selected backend command stream timed out"
                elif vehicle_timed_out:
                    reason = "MAVROS state is stale or vehicle is not armed OFFBOARD"
                elif initial_status_timed_out:
                    reason = "selected backend did not acknowledge the goal"
                else:
                    reason = "selected backend status timed out"
                self._publish_lifecycle(
                    PlannerStatus.FAULT, reason
                )
                rospy.logerr(
                    "[planner_gateway] command gate revoked by watchdog: %s",
                    reason,
                )

    def _publisher_check_callback(self, _event):
        if not self._enforce_unique_output:
            return
        try:
            import rosgraph

            publishers, _subscribers, _services = rosgraph.Master(
                rospy.get_name()
            ).getSystemState()
            nodes = next(
                (names for topic, names in publishers if topic == self._output_topic),
                [],
            )
            unexpected = sorted(set(nodes) - {rospy.get_name()})
        except Exception as exc:
            rospy.logwarn_throttle(
                10.0,
                "[planner_gateway] cannot audit output publishers: %s",
                exc,
            )
            return
        if unexpected:
            with self._lock:
                self._gate.force_close()
                self._fault_reported = self._goal_id > 0
            rospy.logfatal(
                "[planner_gateway] other publishers detected on %s: %s",
                self._output_topic,
                ", ".join(unexpected),
            )
            rospy.signal_shutdown("multiple controller command publishers")


def main():
    rospy.init_node("planner_gateway")
    PlannerGateway()
    rospy.spin()


if __name__ == "__main__":
    main()
