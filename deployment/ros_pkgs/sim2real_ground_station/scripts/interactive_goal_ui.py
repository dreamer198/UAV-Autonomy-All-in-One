#!/usr/bin/env python3
"""Qt safety flows for RViz goals and explicit flight commands.

The ROS action client intentionally lives outside the RViz panel plugin.  It
keeps the safety prompt functional even though every RViz dock is hidden in the
embedded presentation window.  The existing 2D Nav Goal path and the explicit
Takeoff/Land path use separate actions; neither bypasses its onboard safety
server.
"""

import copy
import math
import threading
import time

import actionlib
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import ExtendedState, State
from python_qt_binding.QtCore import QObject, QTimer, Signal, Slot
from python_qt_binding.QtWidgets import (
    QDialog,
    QDialogButtonBox,
    QLabel,
    QVBoxLayout,
)
from sim2real_planning_msgs.msg import (
    FlightCommandAction,
    FlightCommandGoal,
    InteractiveGoalAction,
    InteractiveGoalGoal,
)


ACTION_NAME = "/ground_station/interactive_goal"
FLIGHT_COMMAND_ACTION_NAME = "/ground_station/flight_command"
CANDIDATE_TOPIC = "/ground_station/goal_candidate"
STATE_TIMEOUT_SECONDS = 3.0
FLIGHT_HEIGHT_MIN = 0.5
FLIGHT_HEIGHT_MAX = 2.5

_FLIGHT_STAGE_LABELS = {
    1: "正在验证飞行状态",
    2: "正在解锁无人机",
    3: "正在自动起飞",
    4: "正在进入 OFFBOARD",
    5: "正在请求 AUTO.LAND",
    6: "指令已由机载端确认",
}


def _yaw_degrees(orientation):
    values = (
        float(orientation.x),
        float(orientation.y),
        float(orientation.z),
        float(orientation.w),
    )
    norm = math.sqrt(sum(value * value for value in values))
    if not math.isfinite(norm) or norm < 1e-6:
        return 0.0
    x, y, z, w = (value / norm for value in values)
    return math.degrees(
        math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    )


class InteractiveGoalUi(QObject):
    """Bridge ROS callbacks into guarded, GUI-thread-only operator flows."""

    candidate_for_ui = Signal(object)
    action_finished_for_ui = Signal(bool, str)
    vehicle_state_changed_for_ui = Signal()
    flight_feedback_for_ui = Signal(str, str)
    flight_finished_for_ui = Signal(str, bool, str)

    def __init__(
        self,
        parent=None,
        confirm_callback=None,
        result_callback=None,
        flight_confirm_callback=None,
        flight_result_callback=None,
        status_callback=None,
    ):
        super().__init__(parent)
        self._parent = parent
        self._confirm_callback = confirm_callback
        self._result_callback = result_callback
        self._flight_confirm_callback = flight_confirm_callback
        self._flight_result_callback = flight_result_callback
        self._status_callback = status_callback
        self._lock = threading.Lock()
        self._state = None
        self._state_at = 0.0
        self._extended_state = None
        self._extended_state_at = 0.0
        self._dialog_open = False
        self._action_active = False
        self._flight_dialog_open = False
        self._flight_action_active = False
        self._active_flight_command = ""
        self._takeoff_action = None
        self._land_action = None
        self._cancel_takeoff_action = None
        self._height_control = None

        if not rospy.core.is_initialized():
            rospy.init_node(
                "embedded_ground_station_goal_ui",
                anonymous=True,
                disable_signals=True,
            )
        self._client = actionlib.SimpleActionClient(
            ACTION_NAME, InteractiveGoalAction
        )
        self._flight_client = actionlib.SimpleActionClient(
            FLIGHT_COMMAND_ACTION_NAME, FlightCommandAction
        )
        self._candidate_subscriber = rospy.Subscriber(
            CANDIDATE_TOPIC,
            PoseStamped,
            self._candidate_callback,
            queue_size=1,
            tcp_nodelay=True,
        )
        self._state_subscriber = rospy.Subscriber(
            "/mavros/state",
            State,
            self._state_callback,
            queue_size=1,
            tcp_nodelay=True,
        )
        self._extended_state_subscriber = rospy.Subscriber(
            "/mavros/extended_state",
            ExtendedState,
            self._extended_state_callback,
            queue_size=1,
            tcp_nodelay=True,
        )
        self.candidate_for_ui.connect(self._show_candidate_dialog)
        self.action_finished_for_ui.connect(self._show_action_result)
        self.vehicle_state_changed_for_ui.connect(
            self._refresh_flight_action_state
        )
        self.flight_feedback_for_ui.connect(self._show_flight_feedback)
        self.flight_finished_for_ui.connect(self._show_flight_result)

        # State callbacks stop when the link disappears, so callback-driven
        # updates alone cannot disable buttons after the freshness deadline.
        self._state_freshness_timer = QTimer(self)
        self._state_freshness_timer.setInterval(250)
        self._state_freshness_timer.timeout.connect(
            self._refresh_flight_action_state
        )
        self._state_freshness_timer.start()

    def _candidate_callback(self, message):
        with self._lock:
            # Preserve the established 2D Nav Goal behavior.  The onboard
            # lifecycle lock remains authoritative if a separate flight
            # command happens to be active.
            if self._dialog_open or self._action_active:
                return
            self._dialog_open = True
        self.vehicle_state_changed_for_ui.emit()
        self.candidate_for_ui.emit(copy.deepcopy(message))

    def _state_callback(self, message):
        with self._lock:
            self._state = copy.deepcopy(message)
            self._state_at = time.monotonic()
        self.vehicle_state_changed_for_ui.emit()

    def _extended_state_callback(self, message):
        with self._lock:
            self._extended_state = copy.deepcopy(message)
            self._extended_state_at = time.monotonic()
        self.vehicle_state_changed_for_ui.emit()

    def bind_flight_actions(
        self,
        takeoff_action,
        land_action,
        cancel_takeoff_action=None,
        status_callback=None,
        height_control=None,
    ):
        """Bind toolbar actions while keeping all QWidget access on Qt's thread."""

        self._takeoff_action = takeoff_action
        self._land_action = land_action
        self._cancel_takeoff_action = cancel_takeoff_action
        self._height_control = height_control
        if status_callback is not None:
            self._status_callback = status_callback
        takeoff_action.triggered.connect(
            lambda _checked=False: self.request_takeoff()
        )
        land_action.triggered.connect(
            lambda _checked=False: self.request_land()
        )
        if cancel_takeoff_action is not None:
            cancel_takeoff_action.setVisible(False)
            cancel_takeoff_action.setEnabled(False)
            cancel_takeoff_action.triggered.connect(
                lambda _checked=False: self._cancel_takeoff()
            )
        self._refresh_flight_action_state()

    def _notify(self, level, message, timeout_ms=0):
        """Report operator feedback without opening another top-level window."""

        text = " ".join(str(message or "").split())
        if not text:
            return
        if self._status_callback is not None:
            try:
                self._status_callback(str(level), text, int(timeout_ms))
                return
            except Exception as exc:
                rospy.logerr("Ground-station status callback failed: %s", exc)
        logger = {
            "error": rospy.logerr,
            "warning": rospy.logwarn,
            "success": rospy.loginfo,
            "info": rospy.loginfo,
        }.get(str(level), rospy.loginfo)
        logger("Ground station: %s", text)

    def _state_snapshot(self):
        with self._lock:
            return (
                copy.deepcopy(self._state),
                self._state_at,
                copy.deepcopy(self._extended_state),
                self._extended_state_at,
            )

    def _operation_busy(self):
        with self._lock:
            return (
                self._dialog_open
                or self._action_active
                or self._flight_dialog_open
                or self._flight_action_active
            )

    def _flight_state_rejection(self, command):
        state, state_at, extended_state, extended_state_at = (
            self._state_snapshot()
        )
        now = time.monotonic()
        if (
            state is None
            or now - state_at > STATE_TIMEOUT_SECONDS
            or not state.connected
        ):
            return "等待新鲜且已连接的 MAVROS State。"
        if (
            extended_state is None
            or now - extended_state_at > STATE_TIMEOUT_SECONDS
        ):
            return "等待新鲜的 MAVROS ExtendedState。"

        landed_state = int(extended_state.landed_state)
        if command == "takeoff":
            if state.armed:
                return "无人机已经解锁，不能再次执行起飞。"
            if landed_state != ExtendedState.LANDED_STATE_ON_GROUND:
                return "仅在确认无人机位于地面后允许起飞。"
            return ""

        if not state.armed:
            return "无人机尚未解锁，无需发送降落指令。"
        mode = str(state.mode).upper()
        if mode not in (
            "OFFBOARD",
            "AUTO.TAKEOFF",
            "AUTO.LOITER",
            "AUTO.LAND",
        ):
            return "当前飞行模式 {} 不允许由界面请求降落。".format(
                mode or "未知"
            )
        airborne_states = (
            ExtendedState.LANDED_STATE_IN_AIR,
            ExtendedState.LANDED_STATE_TAKEOFF,
            ExtendedState.LANDED_STATE_LANDING,
        )
        if landed_state not in airborne_states:
            return "仅在确认无人机处于空中或起飞阶段时允许降落。"
        return ""

    def _flight_action_availability(self, command):
        if self._operation_busy():
            return False, "另一项飞行操作正在确认或执行。"
        rejection = self._flight_state_rejection(command)
        if rejection:
            return False, rejection
        if command == "takeoff":
            return True, "自动解锁、起飞并进入 OFFBOARD。"
        return True, "请求 PX4 进入 AUTO.LAND。"

    @Slot()
    def _refresh_flight_action_state(self):
        if self._height_control is not None:
            try:
                self._height_control.setEnabled(not self._operation_busy())
            except RuntimeError:
                self._height_control = None
        if self._takeoff_action is None or self._land_action is None:
            return
        takeoff_ready, takeoff_reason = self._flight_action_availability(
            "takeoff"
        )
        land_ready, land_reason = self._flight_action_availability("land")
        self._takeoff_action.setEnabled(takeoff_ready)
        self._takeoff_action.setToolTip("Takeoff：{}".format(takeoff_reason))
        self._land_action.setEnabled(land_ready)
        self._land_action.setToolTip("Land：{}".format(land_reason))

    @staticmethod
    def _command_title(command):
        return "起飞" if command == "takeoff" else "降落"

    def _configured_height(self):
        if self._height_control is None:
            raise ValueError("统一飞行高度控件尚未就绪。")
        try:
            height = float(self._height_control.value())
        except (AttributeError, RuntimeError, TypeError, ValueError):
            raise ValueError("无法读取统一飞行高度。")
        if (
            not math.isfinite(height)
            or height < FLIGHT_HEIGHT_MIN
            or height > FLIGHT_HEIGHT_MAX
        ):
            raise ValueError(
                "统一飞行高度必须位于 {:.1f}–{:.1f} m。".format(
                    FLIGHT_HEIGHT_MIN, FLIGHT_HEIGHT_MAX
                )
            )
        return height

    def _build_flight_dialog(self, command, height=0.0):
        title = self._command_title(command)
        dialog = QDialog(self._parent)
        dialog.setObjectName("flightCommandDialog")
        dialog.setWindowTitle("确认{}".format(title))
        dialog.setModal(True)
        dialog.setMinimumWidth(440)
        root = QVBoxLayout(dialog)
        root.setContentsMargins(18, 16, 18, 16)
        root.setSpacing(12)

        if command == "takeoff":
            height_label = QLabel(
                "统一飞行高度：{:.2f} m（可在工具栏修改）".format(
                    height
                ),
                dialog,
            )
            height_label.setObjectName("flightCommandHeightLabel")
            root.addWidget(height_label)
            warning_text = (
                "确认后，机载端将再次验证 MAVROS 与落地状态，自动解锁并执行 "
                "AUTO.TAKEOFF 到 {:.2f} m，随后进入 OFFBOARD 保持。\n"
                "请确认桨叶区域和上方航路安全，并保持遥控器可随时接管。"
            ).format(height)
        else:
            warning_text = (
                "确认后，机载端将请求 PX4 进入 AUTO.LAND。Action 成功只表示 "
                "降落模式已得到确认，不表示飞机已经着陆或上锁。\n"
                "请持续观察飞机，直到确认落地并停止旋翼。"
            )

        warning = QLabel(warning_text, dialog)
        warning.setWordWrap(True)
        warning.setStyleSheet(
            "QLabel { color: #fbbf24; background: #422006; "
            "border: 1px solid #92400e; border-radius: 5px; padding: 9px; }"
        )
        root.addWidget(warning)

        buttons = QDialogButtonBox(
            QDialogButtonBox.Ok | QDialogButtonBox.Cancel, dialog
        )
        confirm_button = buttons.button(QDialogButtonBox.Ok)
        cancel_button = buttons.button(QDialogButtonBox.Cancel)
        confirm_button.setText("确认{}".format(title))
        cancel_button.setText("取消")
        confirm_button.setAutoDefault(False)
        confirm_button.setDefault(False)
        cancel_button.setAutoDefault(True)
        cancel_button.setDefault(True)
        buttons.accepted.connect(dialog.accept)
        buttons.rejected.connect(dialog.reject)
        root.addWidget(buttons)
        return dialog

    @Slot()
    def request_takeoff(self):
        self._request_flight_command("takeoff")

    @Slot()
    def request_land(self):
        self._request_flight_command("land")

    def _request_flight_command(self, command):
        ready, reason = self._flight_action_availability(command)
        if not ready:
            self._notify(
                "warning",
                "{}被拒绝：{}".format(
                    self._command_title(command), reason
                ),
                8000,
            )
            return

        with self._lock:
            if (
                self._dialog_open
                or self._action_active
                or self._flight_dialog_open
                or self._flight_action_active
            ):
                return
            self._flight_dialog_open = True
        self.vehicle_state_changed_for_ui.emit()

        try:
            rejection = self._flight_state_rejection(command)
            if rejection:
                raise ValueError(rejection)
            height = self._configured_height() if command == "takeoff" else 0.0
            if not self._flight_client.wait_for_server(rospy.Duration(0.2)):
                raise ValueError("机载起飞/降落服务未连接。")

            dialog = self._build_flight_dialog(command, height)
            if self._flight_confirm_callback is not None:
                self._flight_confirm_callback(
                    dialog, command, height
                )
            if dialog.exec_() != QDialog.Accepted:
                return

            # The modal may have remained open while vehicle state changed.
            rejection = self._flight_state_rejection(command)
            if rejection:
                raise ValueError(rejection)
            height = self._configured_height() if command == "takeoff" else 0.0
            self._send_flight_command(command, height)
        except (ValueError, rospy.ROSException) as exc:
            self._notify(
                "warning",
                "{}被拒绝：{}".format(
                    self._command_title(command), exc
                ),
                8000,
            )
        finally:
            with self._lock:
                self._flight_dialog_open = False
            self.vehicle_state_changed_for_ui.emit()

    def _send_flight_command(self, command, takeoff_height):
        goal = FlightCommandGoal()
        if command == "takeoff":
            goal.command = FlightCommandGoal.TAKEOFF
            goal.takeoff_height = float(takeoff_height)
        else:
            goal.command = FlightCommandGoal.LAND
            goal.takeoff_height = 0.0

        with self._lock:
            self._flight_action_active = True
            self._active_flight_command = command
        self._begin_flight_status(command)
        self.vehicle_state_changed_for_ui.emit()
        try:
            self._flight_client.send_goal(
                goal,
                done_cb=lambda state, result: self._flight_done_callback(
                    command, state, result
                ),
                feedback_cb=lambda feedback: self._flight_feedback_callback(
                    command, feedback
                ),
            )
        except Exception as exc:
            with self._lock:
                self._flight_action_active = False
                self._active_flight_command = ""
            self._end_flight_status()
            self.vehicle_state_changed_for_ui.emit()
            raise ValueError("无法发送飞行指令：{}".format(exc))

    def _begin_flight_status(self, command):
        self._end_flight_status()
        title = self._command_title(command)
        if self._cancel_takeoff_action is not None:
            is_takeoff = command == "takeoff"
            self._cancel_takeoff_action.setVisible(is_takeoff)
            self._cancel_takeoff_action.setEnabled(is_takeoff)
        self._notify(
            "info",
            "{}：正在等待机载端处理指令…".format(title),
        )

    @Slot()
    def _cancel_takeoff(self):
        with self._lock:
            active_takeoff = (
                self._flight_action_active
                and self._active_flight_command == "takeoff"
            )
        if not active_takeoff:
            return
        if self._cancel_takeoff_action is not None:
            self._cancel_takeoff_action.setEnabled(False)
            self._cancel_takeoff_action.setVisible(False)
        self._notify(
            "warning", "取消起飞：正在等待机载端完成安全恢复…"
        )
        self._flight_client.cancel_goal()

    def _end_flight_status(self):
        if self._cancel_takeoff_action is not None:
            self._cancel_takeoff_action.setEnabled(False)
            self._cancel_takeoff_action.setVisible(False)

    def _flight_feedback_callback(self, command, feedback):
        stage = int(getattr(feedback, "stage", 0))
        stage_label = _FLIGHT_STAGE_LABELS.get(stage, "正在处理飞行指令")
        detail = str(getattr(feedback, "message", "") or "").strip()
        message = stage_label if not detail else "{}：{}".format(
            stage_label, detail
        )
        self.flight_feedback_for_ui.emit(command, message)

    @Slot(str, str)
    def _show_flight_feedback(self, command, message):
        with self._lock:
            is_current = (
                self._flight_action_active
                and self._active_flight_command == command
            )
        if is_current:
            self._notify(
                "info",
                "{}：{}".format(self._command_title(command), message),
            )

    def _flight_done_callback(self, command, state, result):
        success = bool(result and result.success and int(state) == 3)
        message = (
            str(result.message)
            if result is not None and result.message
            else "机载起飞/降落服务未返回结果。"
        )
        self.flight_finished_for_ui.emit(command, success, message)

    @Slot(str, bool, str)
    def _show_flight_result(self, command, success, message):
        with self._lock:
            if self._active_flight_command != command:
                return
            self._flight_action_active = False
            self._active_flight_command = ""
        self._end_flight_status()
        self._refresh_flight_action_state()
        if self._flight_result_callback is not None:
            self._flight_result_callback(command, bool(success), str(message))
            return
        if success:
            if command == "land":
                summary = (
                    "降落：AUTO.LAND 已确认，请持续观察直至落地。"
                )
            else:
                summary = "起飞：已完成并进入 OFFBOARD 悬停。"
            self._notify(
                "success", "{} {}".format(summary, message), 7000
            )
        else:
            self._notify(
                "error",
                "{}失败：{}".format(
                    self._command_title(command), message
                ),
                12000,
            )

    def _vehicle_kind(self):
        with self._lock:
            state = copy.deepcopy(self._state)
            state_at = self._state_at
            extended_state = copy.deepcopy(self._extended_state)
            extended_state_at = self._extended_state_at
        now = time.monotonic()
        if (
            state is None
            or now - state_at > STATE_TIMEOUT_SECONDS
            or not state.connected
        ):
            raise ValueError("PX4 连接状态缺失、过期或未连接。")
        if state.armed:
            if state.mode != "OFFBOARD":
                raise ValueError("无人机已解锁但不在 OFFBOARD，拒绝自动切换模式。")
            return "airborne_offboard"
        if (
            extended_state is None
            or now - extended_state_at > STATE_TIMEOUT_SECONDS
            or extended_state.landed_state
            != ExtendedState.LANDED_STATE_ON_GROUND
        ):
            raise ValueError("无法确认无人机处于地面，禁止自动解锁。")
        return "disarmed_ground"

    def _build_ground_goal_dialog(self, target, height):
        dialog = QDialog(self._parent)
        dialog.setObjectName("interactiveGoalDialog")
        dialog.setWindowTitle("确认自动起飞并发送目标")
        dialog.setModal(True)
        dialog.setMinimumWidth(440)
        root = QVBoxLayout(dialog)
        root.setContentsMargins(18, 16, 18, 16)
        root.setSpacing(12)

        target_label = QLabel(
            (
                "目标位置：X={:.2f} m  Y={:.2f} m  Z={:.2f} m  "
                "航向={:.1f}°"
            ).format(
                target.pose.position.x,
                target.pose.position.y,
                height,
                _yaw_degrees(target.pose.orientation),
            ),
            dialog,
        )
        target_label.setWordWrap(True)
        root.addWidget(target_label)

        height_label = QLabel(
            "统一飞行高度：{:.2f} m（可在工具栏修改）".format(height),
            dialog,
        )
        height_label.setObjectName("interactiveGoalHeightLabel")
        root.addWidget(height_label)

        warning = QLabel(
            "无人机当前未解锁且位于地面。确认后将自动解锁，以 "
            "AUTO.TAKEOFF 起飞到统一高度，再进入 OFFBOARD 飞向目标。\n"
            "请确认飞行区域安全，并保持遥控器可随时接管。",
            dialog,
        )
        warning.setWordWrap(True)
        warning.setStyleSheet(
            "QLabel { color: #fbbf24; background: #422006; "
            "border: 1px solid #92400e; border-radius: 5px; padding: 9px; }"
        )
        root.addWidget(warning)

        buttons = QDialogButtonBox(
            QDialogButtonBox.Ok | QDialogButtonBox.Cancel, dialog
        )
        send_button = buttons.button(QDialogButtonBox.Ok)
        cancel_button = buttons.button(QDialogButtonBox.Cancel)
        send_button.setText("确认起飞并发送")
        cancel_button.setText("取消")
        send_button.setAutoDefault(False)
        send_button.setDefault(False)
        cancel_button.setAutoDefault(True)
        cancel_button.setDefault(True)
        buttons.accepted.connect(dialog.accept)
        buttons.rejected.connect(dialog.reject)
        root.addWidget(buttons)
        return dialog

    def _send_interactive_goal(self, target, vehicle_kind, height):
        target.header.stamp = rospy.Time.now()
        target.header.frame_id = "world"
        target.pose.position.z = height
        goal = InteractiveGoalGoal()
        goal.target = target
        goal.takeoff_height = height
        goal.auto_arm_if_grounded = vehicle_kind == "disarmed_ground"
        with self._lock:
            self._action_active = True
        self.vehicle_state_changed_for_ui.emit()
        try:
            self._client.send_goal(goal, done_cb=self._done_callback)
        except Exception as exc:
            with self._lock:
                self._action_active = False
            raise ValueError("无法发送目标请求：{}".format(exc))
        self._notify("info", "目标：正在等待机载端校验…")

    @Slot(object)
    def _show_candidate_dialog(self, target):
        try:
            vehicle_kind = self._vehicle_kind()
            height = self._configured_height()
            if not self._client.wait_for_server(rospy.Duration(0.2)):
                raise ValueError("机载目标服务未连接。")

            if vehicle_kind == "disarmed_ground":
                dialog = self._build_ground_goal_dialog(target, height)
                if self._confirm_callback is not None:
                    self._confirm_callback(
                        dialog,
                        target,
                        vehicle_kind,
                        height,
                        height,
                    )
                if dialog.exec_() != QDialog.Accepted:
                    return
                # The modal may have remained open while vehicle state changed.
                vehicle_kind = self._vehicle_kind()
                height = self._configured_height()
            else:
                self._notify(
                    "info",
                    (
                        "目标：使用统一高度 {:.2f} m，正在提交…"
                    ).format(height),
                )
            self._send_interactive_goal(target, vehicle_kind, height)
        except (ValueError, rospy.ROSException) as exc:
            self._notify(
                "warning", "目标发送被拒绝：{}".format(exc), 10000
            )
        finally:
            with self._lock:
                self._dialog_open = False
            self.vehicle_state_changed_for_ui.emit()

    def _done_callback(self, state, result):
        success = bool(result and result.success and int(state) == 3)
        message = (
            str(result.message)
            if result is not None and result.message
            else "机载目标服务未返回结果。"
        )
        with self._lock:
            self._action_active = False
        self.action_finished_for_ui.emit(success, message)

    @Slot(bool, str)
    def _show_action_result(self, success, message):
        self._refresh_flight_action_state()
        if self._result_callback is not None:
            self._result_callback(bool(success), str(message))
            return
        if success:
            self._notify(
                "success",
                (
                    "目标：规划器已接受请求（不表示已经到达）。 {}"
                ).format(message),
                7000,
            )
        else:
            self._notify(
                "error", "目标执行失败：{}".format(message), 12000
            )

    @Slot()
    def shutdown(self):
        self._state_freshness_timer.stop()
        self._end_flight_status()
        # Closing a visualization client must never preempt a lifecycle command
        # that the operator already confirmed and handed to the onboard server.
        for subscriber in (
            self._candidate_subscriber,
            self._state_subscriber,
            self._extended_state_subscriber,
        ):
            try:
                subscriber.unregister()
            except Exception:
                pass
