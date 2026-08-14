#!/usr/bin/env python3
"""Qt behavior checks for non-modal embedded-RViz operator feedback."""

import os
import sys
import tempfile
import time
import unittest
from pathlib import Path
from unittest import mock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

SCRIPT_DIRECTORY = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPT_DIRECTORY))

from python_qt_binding.QtCore import QEventLoop, QSettings, QTimer
from python_qt_binding.QtWidgets import (
    QAction,
    QApplication,
    QDoubleSpinBox,
    QLabel,
    QMainWindow,
    QToolBar,
    QWidget,
)

import interactive_goal_ui
from embedded_rviz import (
    FLIGHT_HEIGHT_DEFAULT,
    FLIGHT_HEIGHT_SETTINGS_KEY,
    ToolbarStatusPresenter,
    UnifiedFlightHeightControl,
    install_flight_command_actions,
)


class _FakeActionClient:
    def __init__(self):
        self.cancel_count = 0
        self.goals = []

    def wait_for_server(self, _timeout):
        return True

    def send_goal(self, goal, **callbacks):
        self.goals.append((goal, callbacks))

    def cancel_goal(self):
        self.cancel_count += 1


class _FakeSubscriber:
    def unregister(self):
        pass


class _FakeGoalUi:
    def __init__(self):
        self.bound = None
        self.bound_keywords = None

    def bind_flight_actions(self, *arguments, **keywords):
        self.bound = arguments
        self.bound_keywords = keywords


class NonModalStatusTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    @staticmethod
    def wait(milliseconds):
        loop = QEventLoop()
        QTimer.singleShot(milliseconds, loop.quit)
        loop.exec_()

    def test_toolbar_status_is_compact_colored_and_timed(self):
        toolbar = QToolBar()
        following = QAction("Publish Point", toolbar)
        toolbar.addAction(following)
        presenter = ToolbarStatusPresenter(toolbar, following)
        status = toolbar.findChild(QLabel, "groundStationStatusLabel")

        self.assertIsNotNone(status)
        self.assertTrue(status.isHidden())
        presenter.show(
            "success",
            "目标：规划器已接受请求（不表示已经到达）。",
            20,
        )
        self.assertFalse(status.isHidden())
        self.assertIn("目标", status.text())
        self.assertIn("已接受", status.toolTip())
        self.assertIn("#16a34a", status.styleSheet())
        self.wait(50)
        self.assertTrue(status.isHidden())

        presenter.show("error", "目标执行失败：测试故障", 0)
        self.assertFalse(status.isHidden())
        self.assertIn("#dc2626", status.styleSheet())
        self.wait(20)
        self.assertFalse(status.isHidden())
        presenter.clear()
        self.assertTrue(status.isHidden())

    def test_unified_height_defaults_to_one_meter_and_persists(self):
        with tempfile.TemporaryDirectory() as directory:
            settings_path = str(Path(directory) / "height.ini")
            settings = QSettings(settings_path, QSettings.IniFormat)
            toolbar = QToolBar()
            control = UnifiedFlightHeightControl(
                toolbar, settings=settings
            )

            self.assertAlmostEqual(
                control.spin_box.value(), FLIGHT_HEIGHT_DEFAULT
            )
            self.assertAlmostEqual(control.spin_box.minimum(), 0.5)
            self.assertAlmostEqual(control.spin_box.maximum(), 2.5)

            control.spin_box.setValue(1.4)
            settings.sync()
            restored_settings = QSettings(
                settings_path, QSettings.IniFormat
            )
            restored_toolbar = QToolBar()
            restored = UnifiedFlightHeightControl(
                restored_toolbar, settings=restored_settings
            )
            self.assertAlmostEqual(restored.spin_box.value(), 1.4)

            restored_settings.setValue(FLIGHT_HEIGHT_SETTINGS_KEY, "nan")
            restored_settings.sync()
            fallback_toolbar = QToolBar()
            fallback = UnifiedFlightHeightControl(
                fallback_toolbar,
                settings=QSettings(settings_path, QSettings.IniFormat),
            )
            self.assertAlmostEqual(
                fallback.spin_box.value(), FLIGHT_HEIGHT_DEFAULT
            )

    def test_flight_actions_and_status_follow_2d_goal(self):
        frame = QMainWindow()
        toolbar = QToolBar(frame)
        frame.addToolBar(toolbar)
        goal = QAction("2D Nav Goal", toolbar)
        publish = QAction("Publish Point", toolbar)
        toolbar.addAction(goal)
        toolbar.addAction(publish)
        goal_ui = _FakeGoalUi()

        takeoff, land = install_flight_command_actions(frame, goal_ui)
        actions = toolbar.actions()
        cancel_takeoff = frame.findChild(
            QAction, "groundStationCancelTakeoffAction"
        )
        height_label = toolbar.findChild(
            QLabel, "groundStationFlightHeightLabel"
        )
        height_control = toolbar.findChild(
            QDoubleSpinBox, "groundStationFlightHeightSpinBox"
        )
        status = toolbar.findChild(QLabel, "groundStationStatusLabel")
        height_label_action = next(
            action
            for action in actions
            if toolbar.widgetForAction(action) is height_label
        )
        height_control_action = next(
            action
            for action in actions
            if toolbar.widgetForAction(action) is height_control
        )

        self.assertLess(actions.index(goal), actions.index(height_label_action))
        self.assertLess(
            actions.index(height_label_action),
            actions.index(height_control_action),
        )
        self.assertLess(actions.index(height_control_action), actions.index(takeoff))
        self.assertLess(actions.index(takeoff), actions.index(land))
        self.assertLess(actions.index(land), actions.index(cancel_takeoff))
        self.assertLess(actions.index(cancel_takeoff), actions.index(publish))
        self.assertFalse(cancel_takeoff.isVisible())
        self.assertIsNotNone(status)
        self.assertEqual(goal_ui.bound[:3], (takeoff, land, cancel_takeoff))
        self.assertTrue(callable(goal_ui.bound_keywords["status_callback"]))
        self.assertIs(
            goal_ui.bound_keywords["height_control"], height_control
        )
        frame.deleteLater()

    def test_flight_status_uses_toolbar_and_keeps_takeoff_cancel(self):
        parent = QWidget()
        toolbar = QToolBar(parent)
        takeoff = QAction("Takeoff", toolbar)
        land = QAction("Land", toolbar)
        cancel_takeoff = QAction("Cancel Takeoff", toolbar)
        height_control = QDoubleSpinBox(toolbar)
        height_control.setRange(0.5, 2.5)
        height_control.setValue(1.0)
        notifications = []
        goal_client = _FakeActionClient()
        flight_client = _FakeActionClient()

        with mock.patch.object(
            interactive_goal_ui.rospy.core,
            "is_initialized",
            return_value=True,
        ), mock.patch.object(
            interactive_goal_ui.rospy,
            "Subscriber",
            return_value=_FakeSubscriber(),
        ), mock.patch.object(
            interactive_goal_ui.actionlib,
            "SimpleActionClient",
            side_effect=(goal_client, flight_client),
        ):
            ui = interactive_goal_ui.InteractiveGoalUi(parent)

        ui.bind_flight_actions(
            takeoff,
            land,
            cancel_takeoff,
            lambda level, message, timeout: notifications.append(
                (level, message, timeout)
            ),
            height_control,
        )
        baseline_windows = set(QApplication.topLevelWidgets())
        with ui._lock:
            ui._flight_action_active = True
            ui._active_flight_command = "takeoff"
        ui._begin_flight_status("takeoff")

        self.assertTrue(cancel_takeoff.isVisible())
        self.assertTrue(cancel_takeoff.isEnabled())
        self.assertEqual(set(QApplication.topLevelWidgets()), baseline_windows)
        self.assertEqual(notifications[-1][0], "info")

        ui._cancel_takeoff()
        self.assertFalse(cancel_takeoff.isVisible())
        self.assertEqual(flight_client.cancel_count, 1)
        self.assertEqual(notifications[-1][0], "warning")

        with ui._lock:
            ui._flight_action_active = True
            ui._active_flight_command = "takeoff"
        ui._show_flight_result("takeoff", True, "OFFBOARD confirmed")
        self.assertEqual(notifications[-1][0], "success")
        self.assertIn("OFFBOARD", notifications[-1][1])
        self.assertEqual(set(QApplication.topLevelWidgets()), baseline_windows)

        ui._show_action_result(True, "planner accepted")
        self.assertEqual(notifications[-1][0], "success")
        self.assertIn("不表示已经到达", notifications[-1][1])
        self.assertEqual(set(QApplication.topLevelWidgets()), baseline_windows)
        ui.shutdown()
        parent.deleteLater()

    def test_takeoff_and_goal_share_the_toolbar_height(self):
        parent = QWidget()
        toolbar = QToolBar(parent)
        takeoff = QAction("Takeoff", toolbar)
        land = QAction("Land", toolbar)
        height_control = QDoubleSpinBox(toolbar)
        height_control.setRange(0.5, 2.5)
        height_control.setValue(1.2)
        goal_client = _FakeActionClient()
        flight_client = _FakeActionClient()

        def accept_flight(dialog, _command, supplied_height):
            self.assertAlmostEqual(supplied_height, 1.2)
            QTimer.singleShot(0, dialog.accept)

        with mock.patch.object(
            interactive_goal_ui.rospy.core,
            "is_initialized",
            return_value=True,
        ), mock.patch.object(
            interactive_goal_ui.rospy,
            "Subscriber",
            return_value=_FakeSubscriber(),
        ), mock.patch.object(
            interactive_goal_ui.actionlib,
            "SimpleActionClient",
            side_effect=(goal_client, flight_client),
        ):
            ui = interactive_goal_ui.InteractiveGoalUi(
                parent,
                flight_confirm_callback=accept_flight,
            )

        ui.bind_flight_actions(
            takeoff,
            land,
            height_control=height_control,
        )
        state = interactive_goal_ui.State()
        state.connected = True
        state.armed = False
        extended_state = interactive_goal_ui.ExtendedState()
        extended_state.landed_state = (
            interactive_goal_ui.ExtendedState.LANDED_STATE_ON_GROUND
        )
        with ui._lock:
            ui._state = state
            ui._state_at = time.monotonic()
            ui._extended_state = extended_state
            ui._extended_state_at = time.monotonic()
        ui.request_takeoff()

        self.assertEqual(len(flight_client.goals), 1)
        flight_goal = flight_client.goals[0][0]
        self.assertEqual(
            flight_goal.command,
            interactive_goal_ui.FlightCommandGoal.TAKEOFF,
        )
        self.assertAlmostEqual(flight_goal.takeoff_height, 1.2)

        # Complete the mocked Takeoff so the same UI may accept a goal.
        with ui._lock:
            ui._flight_action_active = False
            ui._active_flight_command = ""
            state.armed = True
            state.mode = "OFFBOARD"
            ui._state = state
            ui._state_at = time.monotonic()
            extended_state.landed_state = (
                interactive_goal_ui.ExtendedState.LANDED_STATE_IN_AIR
            )
            ui._extended_state = extended_state
            ui._extended_state_at = time.monotonic()
            ui._dialog_open = True
        height_control.setEnabled(True)
        target = interactive_goal_ui.PoseStamped()
        target.pose.position.x = 2.0
        target.pose.position.y = -1.0
        target.pose.orientation.w = 1.0
        baseline_windows = set(QApplication.topLevelWidgets())
        with mock.patch.object(
            interactive_goal_ui.rospy.Time,
            "now",
            return_value=interactive_goal_ui.rospy.Time(1),
        ):
            ui._show_candidate_dialog(target)

        self.assertEqual(set(QApplication.topLevelWidgets()), baseline_windows)
        self.assertEqual(len(goal_client.goals), 1)
        interactive_goal = goal_client.goals[0][0]
        self.assertAlmostEqual(interactive_goal.target.pose.position.z, 1.2)
        self.assertAlmostEqual(interactive_goal.takeoff_height, 1.2)
        self.assertFalse(interactive_goal.auto_arm_if_grounded)
        ui.shutdown()
        parent.deleteLater()

    def test_ground_goal_confirmation_has_no_second_height_input(self):
        parent = QWidget()
        toolbar = QToolBar(parent)
        takeoff = QAction("Takeoff", toolbar)
        land = QAction("Land", toolbar)
        height_control = QDoubleSpinBox(toolbar)
        height_control.setRange(0.5, 2.5)
        height_control.setValue(1.0)
        goal_client = _FakeActionClient()
        flight_client = _FakeActionClient()

        def accept_goal(
            dialog,
            _target,
            vehicle_kind,
            goal_height,
            takeoff_height,
        ):
            self.assertEqual(vehicle_kind, "disarmed_ground")
            self.assertAlmostEqual(goal_height, 1.0)
            self.assertAlmostEqual(takeoff_height, 1.0)
            self.assertEqual(dialog.findChildren(QDoubleSpinBox), [])
            QTimer.singleShot(0, dialog.accept)

        with mock.patch.object(
            interactive_goal_ui.rospy.core,
            "is_initialized",
            return_value=True,
        ), mock.patch.object(
            interactive_goal_ui.rospy,
            "Subscriber",
            return_value=_FakeSubscriber(),
        ), mock.patch.object(
            interactive_goal_ui.actionlib,
            "SimpleActionClient",
            side_effect=(goal_client, flight_client),
        ):
            ui = interactive_goal_ui.InteractiveGoalUi(
                parent,
                confirm_callback=accept_goal,
            )

        ui.bind_flight_actions(
            takeoff,
            land,
            height_control=height_control,
        )
        state = interactive_goal_ui.State()
        state.connected = True
        state.armed = False
        extended_state = interactive_goal_ui.ExtendedState()
        extended_state.landed_state = (
            interactive_goal_ui.ExtendedState.LANDED_STATE_ON_GROUND
        )
        with ui._lock:
            ui._state = state
            ui._state_at = time.monotonic()
            ui._extended_state = extended_state
            ui._extended_state_at = time.monotonic()
            ui._dialog_open = True
        target = interactive_goal_ui.PoseStamped()
        target.pose.orientation.w = 1.0
        with mock.patch.object(
            interactive_goal_ui.rospy.Time,
            "now",
            return_value=interactive_goal_ui.rospy.Time(1),
        ):
            ui._show_candidate_dialog(target)

        self.assertEqual(len(goal_client.goals), 1)
        goal = goal_client.goals[0][0]
        self.assertAlmostEqual(goal.target.pose.position.z, 1.0)
        self.assertAlmostEqual(goal.takeoff_height, 1.0)
        self.assertTrue(goal.auto_arm_if_grounded)
        ui.shutdown()
        parent.deleteLater()


if __name__ == "__main__":
    unittest.main()
