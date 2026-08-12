#!/usr/bin/env python3
"""Run a dark, presentation-focused RViz window for the Qt ground station."""

import argparse
import os
import signal
import sys

from rviz import bindings as rviz
from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtGui import QColor, QFont, QFontDatabase, QPalette
from python_qt_binding.QtWidgets import (
    QAction,
    QApplication,
    QDockWidget,
    QLabel,
    QStyle,
    QToolBar,
)

# catkin_install_python uses a relay script.  Resolve this source directory
# explicitly so the adjacent UI module is importable both through rosrun and
# when the launchers copy both scripts into /root.
SCRIPT_DIRECTORY = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIRECTORY not in sys.path:
    sys.path.insert(0, SCRIPT_DIRECTORY)
from interactive_goal_ui import InteractiveGoalUi


TARGET_PANEL_TITLE = "Target Control"
CJK_FONT_FAMILIES = (
    "Noto Sans CJK SC",
    "Noto Sans SC",
    "WenQuanYi Micro Hei",
    "WenQuanYi Zen Hei",
)


def _normalized_action_text(action):
    values = (
        action.text(),
        action.toolTip(),
        action.statusTip(),
        action.objectName(),
    )
    return " ".join(str(value) for value in values).replace("&", "").lower()


def _find_2d_nav_goal_action(frame):
    for toolbar in frame.findChildren(QToolBar):
        for action in toolbar.actions():
            text = _normalized_action_text(action)
            if "2d nav goal" in text or "set goal" in text:
                return toolbar, action
    return None, None


def install_flight_command_actions(frame, goal_ui):
    """Place explicit flight actions immediately after RViz's goal tool."""

    toolbar, goal_action = _find_2d_nav_goal_action(frame)
    if toolbar is None or goal_action is None:
        raise RuntimeError(
            "RViz did not create the 2D Nav Goal toolbar action; refusing "
            "to place flight controls in an ambiguous location."
        )

    takeoff_action = QAction(
        frame.style().standardIcon(QStyle.SP_ArrowUp),
        "Takeoff",
        toolbar,
    )
    takeoff_action.setObjectName("groundStationTakeoffAction")
    takeoff_action.setCheckable(False)
    takeoff_action.setEnabled(False)

    land_action = QAction(
        frame.style().standardIcon(QStyle.SP_ArrowDown),
        "Land",
        toolbar,
    )
    land_action.setObjectName("groundStationLandAction")
    land_action.setCheckable(False)
    land_action.setEnabled(False)

    actions = toolbar.actions()
    goal_index = actions.index(goal_action)
    following_action = (
        actions[goal_index + 1] if goal_index + 1 < len(actions) else None
    )
    if following_action is None:
        toolbar.addAction(takeoff_action)
        toolbar.addAction(land_action)
    else:
        toolbar.insertAction(following_action, takeoff_action)
        toolbar.insertAction(following_action, land_action)

    goal_ui.bind_flight_actions(takeoff_action, land_action)
    return takeoff_action, land_action


def apply_cjk_font(app):
    """Select a font that covers Simplified Chinese inside the container."""

    installed_families = {
        str(family) for family in QFontDatabase().families()
    }
    for family in CJK_FONT_FAMILIES:
        if family not in installed_families:
            continue
        font = QFont(app.font())
        font.setFamily(family)
        app.setFont(font)
        return family
    raise RuntimeError(
        "No Simplified Chinese Qt font is installed; rebuild the "
        "ground-station image with fonts-wqy-microhei."
    )


def apply_dark_theme(app):
    """Match the embedded window to the native ground-station palette."""

    apply_cjk_font(app)
    app.setStyle("Fusion")
    palette = QPalette()
    palette.setColor(QPalette.Window, QColor("#0f172a"))
    palette.setColor(QPalette.WindowText, QColor("#dbeafe"))
    palette.setColor(QPalette.Base, QColor("#0b1220"))
    palette.setColor(QPalette.AlternateBase, QColor("#172033"))
    palette.setColor(QPalette.ToolTipBase, QColor("#172033"))
    palette.setColor(QPalette.ToolTipText, QColor("#f8fafc"))
    palette.setColor(QPalette.Text, QColor("#e2e8f0"))
    palette.setColor(QPalette.Button, QColor("#172033"))
    palette.setColor(QPalette.ButtonText, QColor("#e2e8f0"))
    palette.setColor(QPalette.BrightText, QColor("#fb7185"))
    palette.setColor(QPalette.Highlight, QColor("#2563eb"))
    palette.setColor(QPalette.HighlightedText, QColor("#ffffff"))
    app.setPalette(palette)

    app.setStyleSheet(
        "QMainWindow, QDialog, QWidget { background-color: #0f172a; "
        "color: #dbeafe; }"
        "QToolBar { background: #111c30; border: 0; "
        "border-bottom: 1px solid #26354f; spacing: 4px; padding: 4px 8px; }"
        "QToolButton { background: transparent; color: #cbd5e1; "
        "border: 1px solid transparent; border-radius: 4px; "
        "padding: 5px 9px; }"
        "QToolButton:hover { background: #1e293b; border-color: #334155; }"
        "QToolButton:checked { background: #1d4ed8; color: white; "
        "border-color: #3b82f6; }"
        "QDockWidget { color: #e2e8f0; font-size: 13px; }"
        "QDockWidget::title { background: #172033; padding: 8px; "
        "border-bottom: 1px solid #334155; text-align: left; }"
        "QLabel { color: #cbd5e1; }"
        "QDoubleSpinBox { background: #0b1220; color: #f8fafc; "
        "border: 1px solid #334155; border-radius: 4px; padding: 5px; }"
        "QPushButton { background: #2563eb; color: white; "
        "border: 1px solid #3b82f6; border-radius: 4px; padding: 7px 12px; }"
        "QPushButton:hover { background: #1d4ed8; }"
        "QPushButton:disabled { background: #273449; color: #64748b; "
        "border-color: #334155; }"
        "QMessageBox { background: #0f172a; }"
        "QMenu { background: #111827; color: #e2e8f0; "
        "border: 1px solid #334155; }"
        "QMenu::item:selected { background: #1d4ed8; }"
        "QScrollBar { background: #0f172a; width: 10px; height: 10px; }"
        "QScrollBar::handle { background: #475569; border-radius: 4px; "
        "min-height: 24px; min-width: 24px; }"
    )


def simplify_frame(frame):
    """Remove RViz authoring chrome; 2D Nav Goal opens its own dialog."""

    target_dock = None
    for dock in frame.findChildren(QDockWidget):
        title = str(dock.windowTitle())
        if title == TARGET_PANEL_TITLE:
            target_dock = dock
            frame.addDockWidget(Qt.RightDockWidgetArea, dock)
            dock.setMinimumWidth(260)
            dock.setMaximumWidth(310)
            if os.environ.get("SWARM_RVIZ_SIMULATION") == "1":
                for label in dock.findChildren(QLabel):
                    if "192.168.1.123" in str(label.text()):
                        label.setText(
                            "本地仿真无人机\n"
                            "ROS：127.0.0.1:11311  /  MAVROS system 1"
                        )
        dock.hide()

    toolbar, _goal_action = _find_2d_nav_goal_action(frame)
    if toolbar is None:
        toolbars = frame.findChildren(QToolBar)
        toolbar = toolbars[0] if toolbars else None
    if toolbar is None:
        return
    toolbar.setMovable(False)
    toolbar.setFloatable(False)
    toolbar.setToolButtonStyle(Qt.ToolButtonTextBesideIcon)

def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("config")
    parser.add_argument("--title", default="UAV Embedded RViz")
    parser.add_argument("--session-token", default="standalone")
    args = parser.parse_args(argv)
    if not os.path.isfile(args.config):
        parser.error("RViz config does not exist: {}".format(args.config))

    app = QApplication.instance() or QApplication(sys.argv)
    apply_dark_theme(app)
    frame = rviz.VisualizationFrame()
    frame.setSplashPath("")
    frame.initialize()
    config = rviz.Config()
    reader = rviz.YamlConfigReader()
    reader.readFile(config, args.config)
    if reader.error():
        raise RuntimeError(reader.errorMessage())
    frame.load(config)
    simplify_frame(frame)
    goal_ui = InteractiveGoalUi(frame)
    install_flight_command_actions(frame, goal_ui)
    frame._interactive_goal_ui = goal_ui
    frame.setWindowTitle(args.title)
    # The ground-station host reparents this X11 client into its native map
    # widget.  Bypassing the window manager prevents a decorated top-level
    # frame from being left behind when that parent change occurs.
    frame.setWindowFlags(
        Qt.FramelessWindowHint | Qt.X11BypassWindowManagerHint
    )
    frame.setMenuBar(None)
    frame.setStatusBar(None)
    frame.setHideButtonVisibility(False)
    frame.resize(1100, 760)
    frame.show()

    def localize_tools():
        labels = {
            "Move Camera": "View",
            "Measure": "Measure",
        }
        for toolbar in frame.findChildren(QToolBar):
            for action in toolbar.actions():
                text = str(action.text()).replace("&", "").strip()
                if text in labels:
                    action.setText(labels[text])
                elif text in ("Interact", "Select", "Focus Camera"):
                    action.setVisible(False)

    # RViz may restore tool labels once during its first visible layout pass.
    QTimer.singleShot(500, localize_tools)

    print("RVIZ_XID={}".format(int(frame.winId())), flush=True)

    def quit_application(_signum, _frame):
        app.quit()

    signal.signal(signal.SIGINT, quit_application)
    signal.signal(signal.SIGTERM, quit_application)
    # Python signal handlers are dispatched only when the interpreter regains
    # control from Qt's C++ event loop.  A lightweight timer supplies that
    # checkpoint so SIGTERM from the host reliably closes the adopted window.
    signal_timer = QTimer()
    signal_timer.timeout.connect(lambda: None)
    signal_timer.start(200)
    app.aboutToQuit.connect(goal_ui.shutdown)
    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())
