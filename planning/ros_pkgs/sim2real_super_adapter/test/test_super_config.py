#!/usr/bin/env python3

from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parents[2]
CONFIG_PATH = PACKAGE_ROOT / "config" / "planner.yaml"
PLUGIN_CONTROLLER_CONFIG_PATH = (
    REPOSITORY_ROOT / "planning" / "plugins" / "super" / "controller.yaml"
)
LAUNCH_PATH = PACKAGE_ROOT / "launch" / "super_backend.launch"
ADAPTER_SOURCE_PATH = PACKAGE_ROOT / "src" / "super_backend_adapter_node.cpp"
NATIVE_ROOT = "/planning/backends/super/native/"


class SuperConfigTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))
        cls.launch_root = ET.parse(LAUNCH_PATH).getroot()

    def test_native_interfaces_are_plugin_private(self):
        fsm = self.config["fsm"]
        expected = {
            "click_goal_topic": "goal",
            "cmd_topic": "position_command",
            "mpc_cmd_topic": "polynomial_trajectory",
            "heartbeat_topic": "heartbeat",
            "progress_topic": "progress",
            "effective_goal_topic": "effective_goal",
            "reset_service": "reset",
            "validate_goal_service": "validate_goal",
        }
        for key, suffix in expected.items():
            self.assertEqual(fsm[key], NATIVE_ROOT + suffix)

        ros_callback = self.config["rog_map"]["ros_callback"]
        self.assertEqual(ros_callback["odom_topic"], NATIVE_ROOT + "odom_world")
        self.assertEqual(
            ros_callback["cloud_topic"], NATIVE_ROOT + "cloud_world"
        )

    def test_complete_config_uses_the_common_runtime_override(self):
        launch_args = {
            element.attrib["name"]: element.attrib
            for element in self.launch_root.findall("arg")
        }
        self.assertEqual(
            launch_args["config"]["default"],
            "$(eval optenv('SIM2REAL_PLANNER_CONFIG', '') or "
            "find('sim2real_super_adapter') + '/config/planner.yaml')",
        )

    def test_local_profile_uses_requested_dynamics_and_upstream_map_bounds(self):
        fsm = self.config["fsm"]
        boundary = self.config["traj_opt"]["boundary"]
        rog_map = self.config["rog_map"]
        adapter = self.config["adapter"]
        plugin_controller = yaml.safe_load(
            PLUGIN_CONTROLLER_CONFIG_PATH.read_text(encoding="utf-8")
        )
        self.assertEqual(fsm["click_height"], 1.5)
        self.assertGreater(fsm["close_goal_threshold"], 0.0)
        self.assertLessEqual(
            fsm["close_goal_threshold"], adapter["goal_position_tolerance"]
        )
        self.assertEqual(
            fsm["close_goal_threshold"], adapter["goal_position_tolerance"]
        )
        self.assertEqual(boundary["max_vel"], 2.4)
        self.assertLess(boundary["max_vel"], 3.0)
        self.assertEqual(boundary["max_acc"], 3.0)
        self.assertGreater(boundary["max_acc"], 0.8)
        self.assertEqual(adapter["max_velocity"], boundary["max_vel"])
        self.assertEqual(adapter["max_acceleration"], boundary["max_acc"])
        self.assertGreaterEqual(
            plugin_controller["max_feedforward_acc"], boundary["max_acc"]
        )
        self.assertAlmostEqual(
            plugin_controller["kp_px"] * plugin_controller["kp_vx"], 5.7
        )
        self.assertAlmostEqual(
            plugin_controller["kp_py"] * plugin_controller["kp_vy"], 5.7
        )
        self.assertAlmostEqual(
            plugin_controller["kp_pz"] * plugin_controller["kp_vz"], 4.2
        )
        self.assertEqual(plugin_controller["kd_px"], 0.0)
        self.assertEqual(plugin_controller["kd_py"], 0.0)
        self.assertEqual(plugin_controller["kd_pz"], 0.0)
        self.assertFalse(plugin_controller["align_attitude_with_imu"])
        self.assertTrue(rog_map["map_sliding"]["enable"])
        self.assertFalse(rog_map["ros_callback"]["publish_tf"])
        self.assertEqual(rog_map["virtual_ground_height"], -0.1)
        self.assertEqual(rog_map["virtual_ceil_height"], 3.5)
        native_vertical_margin = rog_map["inflation_resolution"] * (
            1 + rog_map["inflation_step"]
        )
        self.assertGreater(
            1.0,
            rog_map["virtual_ground_height"] + native_vertical_margin,
        )

    def test_adapter_rates_meet_gateway_contract(self):
        adapter = self.config["adapter"]
        self.assertGreaterEqual(adapter["command_rate"], 80.0)
        self.assertGreaterEqual(adapter["status_rate"], 5.0)
        self.assertLessEqual(adapter["native_command_timeout"], 0.08)
        self.assertEqual(adapter["planning_timeout"], 10.0)
        self.assertGreater(
            adapter["planning_timeout"], adapter["native_command_timeout"]
        )
        self.assertGreater(adapter["settle_hold_time"], 0.0)
        self.assertGreater(adapter["reached_hold_time"], 0.0)

    def test_committed_trajectory_visualization_is_adapted_to_common_type(self):
        planner = next(
            node
            for node in self.launch_root.iter("node")
            if node.attrib.get("name") == "planner"
        )
        remaps = {
            remap.attrib["from"]: remap.attrib["to"]
            for remap in planner.findall("remap")
        }
        self.assertEqual(
            remaps["visualization/committed_traj"],
            "/planning/backends/$(arg backend_namespace)/native/"
            "committed_trajectory_viz",
        )
        self.assertNotEqual(
            remaps["visualization/committed_traj"],
            "/planning/viz/backend/trajectory",
        )

    def test_normal_authorization_is_scoped_to_the_complete_goal(self):
        source = ADAPTER_SOURCE_PATH.read_text(encoding="utf-8")
        clear_lifecycle = source[
            source.index("void clearCurrentTrajectory()") :
            source.index("goalCallback(")
        ]
        accept_replacement = source[
            source.index("void acceptNativeTrajectory(") :
            source.index("void nativeCommandCallback(")
        ]
        self.assertIn(
            "goal_has_normal_command_ = false;", clear_lifecycle
        )
        self.assertNotIn(
            "goal_has_normal_command_ = false;", accept_replacement
        )

    def test_close_goal_opens_the_gateway_before_switching_to_hold(self):
        source = ADAPTER_SOURCE_PATH.read_text(encoding="utf-8")
        accept_close_goal = source[
            source.index(
                "void acceptCloseGoalWithoutNativeTrajectoryIfReady()"
            ) : source.index("bool validateNativeTrajectory(")
        ]
        command_timer = source[
            source.index("void commandTimer(") : source.index(
                "void statusTimer("
            )
        ]
        self.assertIn(
            "status_.state = sim2real_planning_msgs::PlannerStatus::ACTIVE;",
            accept_close_goal,
        )
        self.assertIn("status_.armable = true;", accept_close_goal)
        self.assertIn("if (synthetic_close_goal)", command_timer)
        self.assertIn(
            "mode = sim2real_planning_msgs::PlannerCommand::NORMAL;",
            command_timer,
        )
        self.assertIn(
            "command.trajectory_id = accepted_public_trajectory_id_;",
            command_timer,
        )
        clear_synthetic = command_timer.index(
            "synthetic_close_goal_active_ = false;"
        )
        publish_reached = command_timer.index(
            "status_.state = sim2real_planning_msgs::PlannerStatus::REACHED;"
        )
        self.assertLess(clear_synthetic, publish_reached)


if __name__ == "__main__":
    unittest.main()
