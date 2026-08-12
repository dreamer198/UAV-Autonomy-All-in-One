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
from python_qt_binding.QtCore import QObject, Qt, QTimer, Signal, Slot
from python_qt_binding.QtWidgets import (
    QDialog,
    QDialogButtonBox,
    QDoubleSpinBox,
    QFormLayout,
    QLabel,
    QMessageBox,
    QProgressDialog,
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
TAKEOFF_HEIGHT_MIN = 0.5
TAKEOFF_HEIGHT_MAX = 2.5

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
    ):
        super().__init__(parent)
        self._parent = parent
        self._confirm_callback = confirm_callback
        self._result_callback = result_callback
        self._flight_confirm_callback = flight_confirm_callback
        self._flight_result_callback = flight_result_callback
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
        self._flight_progress = None
        self._goal_height = 1.5
        self._takeoff_height = 1.5

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

    def bind_flight_actions(self, takeoff_action, land_action):
        """Bind toolbar actions while keeping all QWidget access on Qt's thread."""

        self._takeoff_action = takeoff_action
        self._land_action = land_action
        takeoff_action.triggered.connect(
            lambda _checked=False: self.request_takeoff()
        )
        land_action.triggered.connect(
            lambda _checked=False: self.request_land()
        )
        self._refresh_flight_action_state()

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

    def _build_flight_dialog(self, command):
        title = self._command_title(command)
        dialog = QDialog(self._parent)
        dialog.setObjectName("flightCommandDialog")
        dialog.setWindowTitle("确认{}".format(title))
        dialog.setModal(True)
        dialog.setMinimumWidth(440)
        root = QVBoxLayout(dialog)
        root.setContentsMargins(18, 16, 18, 16)
        root.setSpacing(12)

        takeoff_height = None
        if command == "takeoff":
            form = QFormLayout()
            takeoff_height = QDoubleSpinBox(dialog)
            takeoff_height.setRange(TAKEOFF_HEIGHT_MIN, TAKEOFF_HEIGHT_MAX)
            takeoff_height.setDecimals(2)
            takeoff_height.setSingleStep(0.1)
            takeoff_height.setValue(self._takeoff_height)
            takeoff_height.setSuffix(" m")
            form.addRow("自动起飞高度", takeoff_height)
            root.addLayout(form)
            warning_text = (
                "确认后，机载端将再次验证 MAVROS 与落地状态，自动解锁并执行 "
                "AUTO.TAKEOFF，随后进入 OFFBOARD 保持。\n"
                "请确认桨叶区域和上方航路安全，并保持遥控器可随时接管。"
            )
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
        return dialog, takeoff_height

    @Slot()
    def request_takeoff(self):
        self._request_flight_command("takeoff")

    @Slot()
    def request_land(self):
        self._request_flight_command("land")

    def _request_flight_command(self, command):
        ready, reason = self._flight_action_availability(command)
        if not ready:
            QMessageBox.warning(
                self._parent,
                "{}被拒绝".format(self._command_title(command)),
                reason,
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
            if not self._flight_client.wait_for_server(rospy.Duration(0.2)):
                raise ValueError("机载起飞/降落服务未连接。")

            dialog, takeoff_height = self._build_flight_dialog(command)
            if self._flight_confirm_callback is not None:
                self._flight_confirm_callback(
                    dialog, command, takeoff_height
                )
            if dialog.exec_() != QDialog.Accepted:
                return

            # The modal may have remained open while vehicle state changed.
            rejection = self._flight_state_rejection(command)
            if rejection:
                raise ValueError(rejection)
            height = 0.0
            if takeoff_height is not None:
                height = float(takeoff_height.value())
                self._takeoff_height = height
            self._send_flight_command(command, height)
        except (ValueError, rospy.ROSException) as exc:
            QMessageBox.warning(
                self._parent,
                "{}被拒绝".format(self._command_title(command)),
                str(exc),
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
        self._open_flight_progress(command)
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
            self._close_flight_progress()
            self.vehicle_state_changed_for_ui.emit()
            raise ValueError("无法发送飞行指令：{}".format(exc))

    def _open_flight_progress(self, command):
        self._close_flight_progress()
        title = self._command_title(command)
        cancel_text = "取消起飞" if command == "takeoff" else ""
        progress = QProgressDialog(
            "正在等待机载端处理{}指令…".format(title),
            cancel_text,
            0,
            0,
            self._parent,
        )
        progress.setObjectName("flightCommandProgress")
        progress.setWindowTitle("正在执行{}".format(title))
        progress.setAutoClose(False)
        progress.setAutoReset(False)
        progress.setMinimumDuration(0)
        progress.setWindowModality(Qt.NonModal)
        if command == "takeoff":
            progress.canceled.connect(self._cancel_takeoff)
        else:
            # Once AUTO.LAND is requested the UI must not imply that closing a
            # dialog can safely switch PX4 back to another mode.
            progress.setCancelButton(None)
        progress.show()
        self._flight_progress = progress

    @Slot()
    def _cancel_takeoff(self):
        with self._lock:
            active_takeoff = (
                self._flight_action_active
                and self._active_flight_command == "takeoff"
            )
        if not active_takeoff:
            return
        if self._flight_progress is not None:
            self._flight_progress.setLabelText(
                "正在取消起飞并等待机载端完成安全恢复…"
            )
            self._flight_progress.setCancelButton(None)
        self._flight_client.cancel_goal()

    def _close_flight_progress(self):
        progress = self._flight_progress
        self._flight_progress = None
        if progress is not None:
            progress.close()
            progress.deleteLater()

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
        if is_current and self._flight_progress is not None:
            self._flight_progress.setLabelText(message)

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
        self._close_flight_progress()
        self._refresh_flight_action_state()
        if self._flight_result_callback is not None:
            self._flight_result_callback(command, bool(success), str(message))
            return
        title = self._command_title(command)
        if success:
            if command == "land":
                result_title = "降落指令已接受"
            else:
                result_title = "起飞流程已完成"
            QMessageBox.information(self._parent, result_title, message)
        else:
            QMessageBox.warning(
                self._parent, "{}失败".format(title), message
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

    @Slot(object)
    def _show_candidate_dialog(self, target):
        try:
            vehicle_kind = self._vehicle_kind()
            if not self._client.wait_for_server(rospy.Duration(0.2)):
                raise ValueError("机载目标服务未连接。")

            dialog = QDialog(self._parent)
            dialog.setObjectName("interactiveGoalDialog")
            dialog.setWindowTitle("发送无人机目标")
            dialog.setModal(True)
            dialog.setMinimumWidth(420)
            root = QVBoxLayout(dialog)
            root.setContentsMargins(18, 16, 18, 16)
            root.setSpacing(12)

            target_label = QLabel(
                "目标位置：X={:.2f} m  Y={:.2f} m  航向={:.1f}°".format(
                    target.pose.position.x,
                    target.pose.position.y,
                    _yaw_degrees(target.pose.orientation),
                ),
                dialog,
            )
            target_label.setWordWrap(True)
            root.addWidget(target_label)

            form = QFormLayout()
            goal_height = QDoubleSpinBox(dialog)
            goal_height.setRange(0.5, 2.5)
            goal_height.setDecimals(2)
            goal_height.setSingleStep(0.1)
            goal_height.setValue(self._goal_height)
            goal_height.setSuffix(" m")
            form.addRow("目标高度", goal_height)

            takeoff_height = QDoubleSpinBox(dialog)
            takeoff_height.setRange(0.5, 2.5)
            takeoff_height.setDecimals(2)
            takeoff_height.setSingleStep(0.1)
            takeoff_height.setValue(self._takeoff_height)
            takeoff_height.setSuffix(" m")
            takeoff_height.setEnabled(vehicle_kind == "disarmed_ground")
            form.addRow("自动起飞高度", takeoff_height)
            root.addLayout(form)

            if vehicle_kind == "disarmed_ground":
                warning_text = (
                    "无人机当前未解锁且位于地面。发送后将自动解锁，"
                    "以 AUTO.TAKEOFF 起飞到设定高度，再进入 OFFBOARD 飞向目标。\n"
                    "请确认飞行区域安全，并保持遥控器可随时接管。"
                )
            else:
                warning_text = "无人机已处于 OFFBOARD；确认后将发送新的飞行目标。"
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
            send_button = buttons.button(QDialogButtonBox.Ok)
            cancel_button = buttons.button(QDialogButtonBox.Cancel)
            send_button.setText("确认发送")
            cancel_button.setText("取消")
            send_button.setAutoDefault(False)
            send_button.setDefault(False)
            cancel_button.setAutoDefault(True)
            cancel_button.setDefault(True)
            buttons.accepted.connect(dialog.accept)
            buttons.rejected.connect(dialog.reject)
            root.addWidget(buttons)

            if self._confirm_callback is not None:
                self._confirm_callback(
                    dialog,
                    target,
                    vehicle_kind,
                    goal_height,
                    takeoff_height,
                )
            if dialog.exec_() != QDialog.Accepted:
                return
            # Recheck state after the operator may have left the modal open.
            vehicle_kind = self._vehicle_kind()
            self._goal_height = goal_height.value()
            self._takeoff_height = takeoff_height.value()
            target.header.stamp = rospy.Time.now()
            target.header.frame_id = "world"
            target.pose.position.z = self._goal_height
            goal = InteractiveGoalGoal()
            goal.target = target
            goal.takeoff_height = self._takeoff_height
            goal.auto_arm_if_grounded = vehicle_kind == "disarmed_ground"
            with self._lock:
                self._action_active = True
            self.vehicle_state_changed_for_ui.emit()
            self._client.send_goal(goal, done_cb=self._done_callback)
        except ValueError as exc:
            QMessageBox.warning(self._parent, "目标发送被拒绝", str(exc))
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
            QMessageBox.information(self._parent, "目标已接受", message)
        else:
            QMessageBox.warning(self._parent, "目标执行失败", message)

    @Slot()
    def shutdown(self):
        self._state_freshness_timer.stop()
        self._close_flight_progress()
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
