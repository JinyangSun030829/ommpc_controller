#!/usr/bin/env python3
"""ROS message-level tests with mocked ROS I/O: no master or flight commands."""
import copy
import importlib.util
from pathlib import Path
import unittest
from unittest.mock import patch

import rospy
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

spec = importlib.util.spec_from_file_location("rviz_quad", Path(__file__).resolve().parents[1] / "traj" / "rviz_quad.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class FakePublisher:
    def __init__(self, topic, message_type, queue_size):
        self.topic = topic
        self.messages = []

    def publish(self, message):
        self.messages.append(copy.deepcopy(message))


class SpeedTextTest(unittest.TestCase):
    def setUp(self):
        self.params = {"~publish_tf": False}
        for target, replacement in (
            ("get_param", lambda name, default: self.params.get(name, default)),
            ("Publisher", FakePublisher), ("Subscriber", lambda *a, **kw: None),
            ("loginfo", lambda *a: None), ("logwarn_throttle", lambda *a: None),
        ):
            mock = patch.object(module.rospy, target, replacement)
            mock.start()
            self.addCleanup(mock.stop)

    def odom(self, x=1.0, speed=(3.0, 4.0, 0.0)):
        m = Odometry()
        m.header.stamp = rospy.Time(10)
        m.pose.pose.position.x = x
        m.pose.pose.position.y = 2
        m.pose.pose.position.z = 3
        m.pose.pose.orientation.w = 1
        m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.linear.z = speed
        return m

    def test_following_text_and_existing_trace(self):
        node = module.DroneTrajectoryPublisher()
        node.odom_callback(self.odom())
        self.assertEqual(node.speed_text_pub.topic, "/uav1/speed_text")
        text = node.speed_text_pub.messages[-1]
        self.assertEqual(text.type, Marker.TEXT_VIEW_FACING)
        self.assertEqual(text.ns, "uav_speed")
        self.assertEqual(text.text, "Speed: 5.00 m/s")
        self.assertEqual(text.header.frame_id, node.world_frame)
        self.assertEqual(text.header.stamp, rospy.Time(10))
        self.assertAlmostEqual(text.pose.position.x, 1.3)
        self.assertAlmostEqual(text.pose.position.z, 3.4)
        self.assertAlmostEqual(text.scale.z, .2)
        self.assertAlmostEqual(text.lifetime.to_sec(), .5)
        self.assertEqual(text.color, node.traj_pub.messages[-1].colors[-1])
        self.assertAlmostEqual(text.color.a, module.COLOR_ALPHA)
        self.assertEqual(node.traj_pub.messages[-1].type, Marker.LINE_STRIP)
        self.assertEqual(len(node.traj_pub.messages[-1].colors), 1)
        node.odom_callback(self.odom(x=4, speed=(0, 0, 0)))
        self.assertEqual(node.speed_text_pub.messages[-1].text, "Speed: 0.00 m/s")
        self.assertAlmostEqual(node.speed_text_pub.messages[-1].pose.position.x, 4.3)
        self.assertEqual(node.speed_text_pub.messages[-1].id, text.id)
        self.assertEqual(node.speed_text_pub.messages[-1].color, node.traj_pub.messages[-1].colors[-1])

    def test_text_matches_trace_across_speed_range_and_dynamic_rescaling(self):
        node = module.DroneTrajectoryPublisher()
        for speed in (0, 1, 2.1, 3.85, 4.9, 5.95, 7, 10):
            node.odom_callback(self.odom(speed=(speed, 0, 0)))
            self.assertEqual(node.speed_text_pub.messages[-1].color,
                             node.traj_pub.messages[-1].colors[-1])
            self.assertEqual(node.speed_text_pub.messages[-1].color,
                             node.get_color_from_speed(speed))
        self.assertGreater(node.max_speed, 7)

    def test_invalid_data_does_not_refresh_label(self):
        node = module.DroneTrajectoryPublisher()
        node.odom_callback(self.odom())
        node.odom_callback(self.odom(speed=(float("nan"), 0, 0)))
        m = self.odom()
        m.pose.pose.orientation.w = 0
        node.odom_callback(m)
        self.assertEqual(len(node.speed_text_pub.messages), 1)
        self.assertEqual(len(node.traj_pub.messages), 1)

    def test_disable_label_without_disabling_trace(self):
        self.params["~speed_text_enabled"] = False
        node = module.DroneTrajectoryPublisher()
        node.odom_callback(self.odom())
        self.assertIsNone(node.speed_text_pub)
        self.assertEqual(len(node.traj_pub.messages), 1)

    def test_custom_parameters(self):
        self.params.update({"~speed_text_offset": [0, -.5, .6], "~speed_text_height": .3,
                            "~speed_text_lifetime": .8, "~speed_text_topic": "/test/speed"})
        node = module.DroneTrajectoryPublisher()
        node.odom_callback(self.odom())
        text = node.speed_text_pub.messages[-1]
        self.assertEqual(node.speed_text_pub.topic, "/test/speed")
        self.assertAlmostEqual(text.pose.position.y, 1.5)
        self.assertAlmostEqual(text.pose.position.z, 3.6)
        self.assertAlmostEqual(text.scale.z, .3)
        self.assertAlmostEqual(text.lifetime.to_sec(), .8)

    def test_invalid_parameters(self):
        for name, value in (("speed_text_height", 0), ("speed_text_lifetime", 0),
                            ("speed_text_offset", [0, 1]), ("speed_text_offset", [0, 0, float("inf")]),
                            ("speed_text_topic", "")):
            with self.subTest(name=name, value=value):
                self.params = {"~publish_tf": False, "~" + name: value}
                with self.assertRaises(ValueError):
                    module.DroneTrajectoryPublisher()

    def test_state_initialized_before_immediate_subscriber_callback(self):
        with patch.object(module.rospy, "Subscriber", lambda topic, message_type, callback, **kw: callback(self.odom())):
            node = module.DroneTrajectoryPublisher()
        self.assertEqual(len(node.speed_text_pub.messages), 1)


if __name__ == "__main__":
    unittest.main()
