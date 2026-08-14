#!/usr/bin/env python3

from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[2]
SE3_SOURCE = (
    PROJECT_ROOT
    / "third_party"
    / "Diff-Planner-PX4"
    / "src"
    / "se3_controller"
    / "src"
    / "se3_ctrl.cpp"
)


class ControllerGatewayContractTest(unittest.TestCase):
    def test_legacy_desired_odometry_cannot_bypass_gateway(self):
        source = SE3_SOURCE.read_text(encoding="utf-8")
        self.assertNotIn(
            'subscribe<nav_msgs::Odometry>("/desire_odom"', source
        )
        self.assertNotIn("DesireOdomCallback", source)

    def test_controller_checks_trajectory_publisher_identity(self):
        source = SE3_SOURCE.read_text(encoding="utf-8")
        self.assertIn("event.getConnectionHeaderPtr()", source)
        self.assertIn('connection_header->find("callerid")', source)
        self.assertIn("caller->second != command_publisher_node_", source)
        self.assertIn('"/planner_gateway"', source)

    def test_controller_discards_old_motion_within_gateway_deadline(self):
        config = yaml.safe_load(
            (PROJECT_ROOT / "common" / "config" / "controller.yaml").read_text(
                encoding="utf-8"
            )
        )
        self.assertLessEqual(
            float(config["trajectory_command_timeout"]), 0.08
        )
        source = SE3_SOURCE.read_text(encoding="utf-8")
        self.assertIn("trajectory_command_timeout_{0.08}", (
            PROJECT_ROOT
            / "third_party"
            / "Diff-Planner-PX4"
            / "src"
            / "se3_controller"
            / "include"
            / "se3_controller"
            / "se3_ctrl.h"
        ).read_text(encoding="utf-8"))
        self.assertIn("requestSafetyHold", source)

    def test_controller_keeps_local_hold_and_planner_feedback_frames_separate(self):
        config = yaml.safe_load(
            (PROJECT_ROOT / "common" / "config" / "controller.yaml").read_text(
                encoding="utf-8"
            )
        )
        self.assertEqual(config["odometry_topic"], "/localization/odom")
        self.assertEqual(
            config["local_odometry_topic"], "/mavros/local_position/odom"
        )

        source = SE3_SOURCE.read_text(encoding="utf-8")
        self.assertIn('msg.header.frame_id != "world"', source)
        self.assertIn('msg->header.frame_id != "map"', source)
        self.assertIn("if (!has_trajectory_after_offboard_)", source)
        self.assertIn(
            "pubLocalPose(local_hold_position_, local_hold_orientation_)",
            source,
        )
        self.assertIn(
            "msg.pose.orientation.w = normalized_orientation.w()", source
        )
        self.assertIn("applyAttitudeHandoff(output)", source)
        self.assertIn("attitudeAlignmentIsStable()", source)

        frame_launch = ET.parse(
            PROJECT_ROOT
            / "deployment"
            / "ros_pkgs"
            / "sim2real_deployment"
            / "launch"
            / "frame_aliases.launch"
        )
        node_names = {
            node.attrib.get("name")
            for node in frame_launch.getroot().findall("node")
        }
        self.assertNotIn("real_world_map_tf", node_names)
        self.assertIn("real_world_odom_tf", node_names)

        real_launch = (PROJECT_ROOT / "launch" / "real.sh").read_text(
            encoding="utf-8"
        )
        self.assertIn("'/real_world_odom_tf'", real_launch)
        self.assertNotIn("'/real_world_map_tf'", real_launch)

    def test_attitude_handoff_has_a_nonzero_safety_duration(self):
        config = yaml.safe_load(
            (PROJECT_ROOT / "common" / "config" / "controller.yaml").read_text(
                encoding="utf-8"
            )
        )
        self.assertGreaterEqual(float(config["attitude_handoff_duration"]), 1.0)
        self.assertLessEqual(
            float(config["max_attitude_alignment_error_deg"]), 10.0
        )
        super_config = yaml.safe_load(
            (
                PROJECT_ROOT / "planning" / "plugins" / "super" / "controller.yaml"
            ).read_text(encoding="utf-8")
        )
        self.assertIs(super_config["align_attitude_with_imu"], True)

    def test_gateway_allows_jitter_on_one_hz_mavros_state(self):
        launch = ET.parse(
            PROJECT_ROOT
            / "planning"
            / "ros_pkgs"
            / "sim2real_planner_manager"
            / "launch"
            / "planner_gateway.launch"
        )
        state_timeout = next(
            element
            for element in launch.getroot().findall("arg")
            if element.attrib.get("name") == "state_timeout"
        )
        self.assertGreaterEqual(float(state_timeout.attrib["default"]), 3.0)
        rate_window = next(
            element
            for element in launch.getroot().findall("arg")
            if element.attrib.get("name") == "rate_window"
        )
        self.assertGreaterEqual(float(rate_window.attrib["default"]), 1.0)

    def test_gateway_does_not_reinterpret_plugin_dynamics_or_map(self):
        source = (
            PROJECT_ROOT
            / "planning"
            / "ros_pkgs"
            / "sim2real_planner_manager"
            / "scripts"
            / "planner_gateway.py"
        ).read_text(encoding="utf-8")
        self.assertNotIn("validate_command_limits", source)

    def test_planner_selection_has_no_implicit_diff_default(self):
        launch_files = (
            PROJECT_ROOT / "common" / "launch" / "planner.launch",
            PROJECT_ROOT / "common" / "launch" / "planning_control.launch",
            PROJECT_ROOT
            / "planning"
            / "ros_pkgs"
            / "sim2real_planner_manager"
            / "launch"
            / "planner_gateway.launch",
        )
        for path in launch_files:
            with self.subTest(path=path):
                root = ET.parse(path).getroot()
                planner_arg = next(
                    element
                    for element in root.findall("arg")
                    if element.attrib.get("name") == "planner_id"
                )
                self.assertNotIn("default", planner_arg.attrib)

        gateway_source = (
            PROJECT_ROOT
            / "planning"
            / "ros_pkgs"
            / "sim2real_planner_manager"
            / "scripts"
            / "planner_gateway.py"
        ).read_text(encoding="utf-8")
        self.assertIn('get_param("~planner_id", "")', gateway_source)
        self.assertIn("no default planner exists", gateway_source)


if __name__ == "__main__":
    unittest.main()
