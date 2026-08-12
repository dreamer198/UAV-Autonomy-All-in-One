#!/usr/bin/env python3

import ast
import os
import unittest


SCRIPT = os.path.abspath(
    os.path.join(
        os.path.dirname(__file__), "..", "scripts", "planner_visualization.py"
    )
)


class PlannerVisualizationContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with open(SCRIPT, "r", encoding="utf-8") as stream:
            cls.source = stream.read()
        cls.tree = ast.parse(cls.source, filename=SCRIPT)

    def test_public_maps_are_rate_limited_and_point_bounded(self):
        self.assertIn('rospy.get_param("~publish_rate", 1.0)', self.source)
        self.assertIn(
            'rospy.get_param("~max_occupancy_points", 20000)', self.source
        )
        self.assertIn(
            'rospy.get_param("~max_inflated_points", 12000)', self.source
        )
        self.assertIn("def bounded_cloud", self.source)
        self.assertIn("stride = int(math.ceil", self.source)
        self.assertIn("def publish_maps", self.source)
        self.assertIn("self.publish_timer = rospy.Timer", self.source)

    def test_public_maps_are_latched_with_single_message_queues(self):
        """Late RViz subscribers receive the last bounded map, not a stream."""
        publisher_assignments = {}
        for node in ast.walk(self.tree):
            if not isinstance(node, ast.Assign) or len(node.targets) != 1:
                continue
            target = node.targets[0]
            if not (
                isinstance(target, ast.Attribute)
                and isinstance(target.value, ast.Name)
                and target.value.id == "self"
                and target.attr in ("occupancy_pub", "inflated_pub")
            ):
                continue
            call = node.value
            if not (
                isinstance(call, ast.Call)
                and isinstance(call.func, ast.Attribute)
                and isinstance(call.func.value, ast.Name)
                and call.func.value.id == "rospy"
                and call.func.attr == "Publisher"
            ):
                continue
            publisher_assignments[target.attr] = {
                keyword.arg: keyword.value for keyword in call.keywords
            }

        self.assertEqual(
            set(publisher_assignments), {"occupancy_pub", "inflated_pub"}
        )
        for name, keywords in publisher_assignments.items():
            with self.subTest(publisher=name):
                self.assertIn("queue_size", keywords)
                self.assertIn("latch", keywords)
                self.assertIsInstance(keywords["queue_size"], ast.Constant)
                self.assertEqual(keywords["queue_size"].value, 1)
                self.assertIsInstance(keywords["latch"], ast.Constant)
                self.assertIs(keywords["latch"].value, True)

    def test_clear_messages_replace_latched_maps(self):
        """A map-ready loss must latch empty maps instead of stale obstacles."""
        clear_function = next(
            node
            for node in ast.walk(self.tree)
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
            and node.name == "clear_maps"
        )
        clear_source = ast.get_source_segment(self.source, clear_function)
        self.assertIn("self.occupancy_pub.publish", clear_source)
        self.assertIn("self.inflated_pub.publish", clear_source)
        self.assertEqual(clear_source.count("self.empty_cloud("), 2)

    def test_map_and_clear_publications_are_serialized_by_the_state_lock(self):
        """A stale pending map cannot overtake the latched empty-map clear."""
        for function_name in ("publish_maps", "clear_maps"):
            function = next(
                node
                for node in ast.walk(self.tree)
                if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
                and node.name == function_name
            )
            lock_blocks = [
                node
                for node in function.body
                if isinstance(node, ast.With)
                and any(
                    isinstance(item.context_expr, ast.Attribute)
                    and isinstance(item.context_expr.value, ast.Name)
                    and item.context_expr.value.id == "self"
                    and item.context_expr.attr == "lock"
                    for item in node.items
                )
            ]
            self.assertEqual(len(lock_blocks), 1, function_name)
            publish_calls = [
                node
                for node in ast.walk(function)
                if isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute)
                and node.func.attr == "publish"
            ]
            locked_publish_calls = [
                node
                for node in ast.walk(lock_blocks[0])
                if isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute)
                and node.func.attr == "publish"
            ]
            self.assertGreater(len(publish_calls), 0, function_name)
            self.assertEqual(
                len(locked_publish_calls), len(publish_calls), function_name
            )

        status_function = next(
            node
            for node in ast.walk(self.tree)
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
            and node.name == "status_callback"
        )
        status_lock = next(
            node for node in status_function.body if isinstance(node, ast.With)
        )
        self.assertTrue(
            any(
                isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute)
                and node.func.attr == "clear_maps"
                for node in ast.walk(status_lock)
            )
        )


if __name__ == "__main__":
    unittest.main()
