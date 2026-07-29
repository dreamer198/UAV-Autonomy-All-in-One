"""ROS-independent state machine protecting the controller command stream."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional


STARTING = 0
READY = 1
PLANNING = 2
ACTIVE = 3
HOLDING = 4
REACHED = 5
FAULT = 6

NORMAL = 0
HOLD = 1
BRAKE = 2


@dataclass(frozen=True)
class GateConfig:
    backend_id: str
    session_id: str
    require_offboard: bool = True
    # MAVROS publishes /mavros/state at roughly 1 Hz in the PX4 simulation.
    # Keep enough margin for normal scheduler jitter while the odometry and
    # trajectory streams retain their independent, much tighter watchdogs.
    state_timeout_sec: float = 3.0
    status_timeout_sec: float = 1.0
    command_timeout_sec: float = 0.25

    def __post_init__(self):
        if not self.backend_id or not self.session_id:
            raise ValueError("backend_id and session_id must be non-empty")
        for name in (
            "state_timeout_sec",
            "status_timeout_sec",
            "command_timeout_sec",
        ):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError("{} must be finite and positive".format(name))


@dataclass(frozen=True)
class StatusSnapshot:
    session_id: str
    backend_id: str
    goal_id: int
    trajectory_id: int
    state: int
    odom_ready: bool
    map_ready: bool
    armable: bool
    received_at: float


@dataclass(frozen=True)
class CommandDecision:
    accepted: bool
    reason: str
    opened_gate: bool = False

    @classmethod
    def accept(cls, opened_gate=False):
        return cls(True, "", opened_gate)

    @classmethod
    def reject(cls, reason):
        return cls(False, reason, False)


class CommandGate:
    """Correlate every backend command with vehicle, goal, and status state.

    Times use a caller-supplied monotonic clock. This keeps the safety logic
    deterministic and testable without rospy or a running ROS master.
    """

    def __init__(self, config: GateConfig):
        self.config = config
        self.current_goal_id = 0
        self._vehicle_received_at: Optional[float] = None
        self._vehicle_connected = False
        self._vehicle_armed = False
        self._vehicle_mode = ""
        self._status: Optional[StatusSnapshot] = None
        self._opened = False
        self._revoked = True
        self._last_trajectory_id = 0
        self._last_command_at: Optional[float] = None

    @property
    def is_open(self):
        return self._opened

    @property
    def is_revoked(self):
        return self._revoked

    @property
    def last_trajectory_id(self):
        return self._last_trajectory_id

    @property
    def last_command_at(self):
        return self._last_command_at

    @property
    def status(self):
        return self._status

    def vehicle_ready(self, now: float) -> bool:
        return self._vehicle_ready(now)

    def force_close(self, revoke: bool = True) -> None:
        self._opened = False
        if revoke:
            self._revoked = True

    def begin_goal(self, goal_id: int) -> None:
        if not isinstance(goal_id, int) or isinstance(goal_id, bool) or goal_id <= 0:
            raise ValueError("goal_id must be a positive integer")
        if goal_id <= self.current_goal_id:
            raise ValueError("goal_id must increase monotonically")
        self.current_goal_id = goal_id
        self._status = None
        self._opened = False
        self._revoked = False
        self._last_trajectory_id = 0
        self._last_command_at = None

    def cancel_goal(self) -> None:
        self._opened = False
        self._revoked = True
        self._status = None
        self._last_trajectory_id = 0
        self._last_command_at = None

    def update_vehicle(
        self,
        connected: bool,
        armed: bool,
        mode: str,
        received_at: float,
    ) -> None:
        if not math.isfinite(received_at):
            raise ValueError("received_at must be finite")
        self._vehicle_connected = bool(connected)
        self._vehicle_armed = bool(armed)
        self._vehicle_mode = str(mode)
        self._vehicle_received_at = received_at
        if (
            self.current_goal_id > 0
            and self.config.require_offboard
            and not self._vehicle_ready(received_at)
        ):
            self._opened = False
            self._revoked = True

    def _vehicle_ready(self, now: float) -> bool:
        if not self.config.require_offboard:
            return True
        return bool(
            self._vehicle_received_at is not None
            and now - self._vehicle_received_at <= self.config.state_timeout_sec
            and self._vehicle_connected
            and self._vehicle_armed
            and self._vehicle_mode == "OFFBOARD"
        )

    def update_status(self, status: StatusSnapshot) -> CommandDecision:
        if not math.isfinite(status.received_at):
            return CommandDecision.reject("status receive time is non-finite")
        if status.session_id != self.config.session_id:
            return CommandDecision.reject("status session does not match")
        if status.backend_id != self.config.backend_id:
            return CommandDecision.reject("status backend does not match")
        if status.goal_id != self.current_goal_id:
            return CommandDecision.reject("status goal does not match")
        if status.goal_id <= 0:
            return CommandDecision.reject("status has no active goal")
        if status.trajectory_id < 0:
            return CommandDecision.reject("status trajectory id is invalid")
        if status.state not in {
            STARTING,
            READY,
            PLANNING,
            ACTIVE,
            HOLDING,
            REACHED,
            FAULT,
        }:
            return CommandDecision.reject("status lifecycle value is invalid")

        previous = self._status
        if (
            previous is not None
            and status.trajectory_id > 0
            and previous.trajectory_id > 0
            and status.trajectory_id < previous.trajectory_id
        ):
            return CommandDecision.reject("status trajectory id moved backwards")
        self._status = status
        if not status.odom_ready or not status.map_ready or status.state == FAULT:
            self._opened = False
            self._revoked = True
        elif status.state not in {ACTIVE, HOLDING, REACHED}:
            # PLANNING is a normal in-goal replan transition: close until the
            # new status/trajectory pair arrives, without revoking the goal.
            self._opened = False
        return CommandDecision.accept()

    def evaluate_command(
        self,
        *,
        session_id: str,
        backend_id: str,
        goal_id: int,
        trajectory_id: int,
        mode: int,
        values_finite: bool,
        shape_valid: bool,
        received_at: float,
        header_age_sec: Optional[float] = None,
    ) -> CommandDecision:
        if not math.isfinite(received_at):
            return CommandDecision.reject("command receive time is non-finite")
        if self._revoked:
            return CommandDecision.reject(
                "goal authorization was revoked; a new goal is required"
            )
        if session_id != self.config.session_id:
            return CommandDecision.reject("command session does not match")
        if backend_id != self.config.backend_id:
            return CommandDecision.reject("command backend does not match")
        if goal_id <= 0 or goal_id != self.current_goal_id:
            return CommandDecision.reject("command goal does not match")
        if trajectory_id <= 0:
            return CommandDecision.reject("command trajectory id is invalid")
        if mode not in {NORMAL, HOLD, BRAKE}:
            return CommandDecision.reject("command mode is invalid")
        if not values_finite:
            return CommandDecision.reject("command contains non-finite values")
        if not shape_valid:
            return CommandDecision.reject("command point shape is invalid")
        if header_age_sec is not None:
            if not math.isfinite(header_age_sec):
                return CommandDecision.reject("command stamp age is non-finite")
            if (
                header_age_sec < -self.config.command_timeout_sec
                or header_age_sec > self.config.command_timeout_sec
            ):
                return CommandDecision.reject("command stamp is stale or in the future")
        if not self._vehicle_ready(received_at):
            self._opened = False
            self._revoked = True
            return CommandDecision.reject("vehicle is not connected armed OFFBOARD")

        status = self._status
        if status is None:
            return CommandDecision.reject("no matching planner status")
        if received_at - status.received_at > self.config.status_timeout_sec:
            self._opened = False
            self._revoked = True
            return CommandDecision.reject("planner status is stale")
        if not status.odom_ready or not status.map_ready:
            self._opened = False
            self._revoked = True
            return CommandDecision.reject("planner inputs are not ready")
        if status.state not in {ACTIVE, HOLDING, REACHED}:
            self._opened = False
            return CommandDecision.reject(
                "planner is not active, holding, or reached"
            )
        if status.trajectory_id != trajectory_id:
            return CommandDecision.reject(
                "command trajectory does not match planner status"
            )
        if trajectory_id < self._last_trajectory_id:
            return CommandDecision.reject("command trajectory id moved backwards")

        opened_now = False
        if trajectory_id > self._last_trajectory_id:
            if mode == NORMAL:
                # Every replacement motion trajectory must independently be
                # declared armable.
                if status.state != ACTIVE or not status.armable:
                    return CommandDecision.reject(
                        "new normal trajectory is not active and armable"
                    )
                self._opened = True
                opened_now = True
            elif not self._opened:
                # A stop trajectory may replace an authorized motion
                # trajectory, but it can never authorize a goal by itself.
                return CommandDecision.reject(
                    "hold/brake trajectory cannot open a closed gate"
                )
        elif not self._opened:
            if mode != NORMAL or status.state != ACTIVE or not status.armable:
                return CommandDecision.reject("command gate has not been armed")
            self._opened = True
            opened_now = True

        if mode == NORMAL:
            if status.state != ACTIVE or not status.armable:
                self._opened = False
                return CommandDecision.reject(
                    "normal command lacks active armable status"
                )
        elif not self._opened:
            return CommandDecision.reject(
                "hold/brake command cannot open a closed gate"
            )
        elif status.state == REACHED and mode != HOLD:
            return CommandDecision.reject(
                "only hold commands are accepted after goal reached"
            )

        self._last_trajectory_id = trajectory_id
        self._last_command_at = received_at
        return CommandDecision.accept(opened_gate=opened_now)
