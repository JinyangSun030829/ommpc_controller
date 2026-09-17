#!/usr/bin/env python3
"""Publish a speed-coloured UAV trajectory, live speed label and optional TF.

Odometry pose coordinates are treated as local-world/ENU coordinates. The
configured ``world_frame`` labels that same coordinate system; this script does
not transform positions between different world origins.
"""

import math
from collections import deque

import rospy
import tf2_ros
from geometry_msgs.msg import Point, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker


DEFAULT_PARAMETERS = {
    "odom_topic": "/uav1/mavros/local_position/odom",
    "trajectory_topic": "/uav1/trajectory_marker",
    "world_frame": "world",
    "body_frame": "",  # Empty: odom.child_frame_id, or base_link as fallback.
    "publish_tf": True,  # Disable if another node owns the same TF.
    "max_points": 5000,
    "line_width": 0.05,
    "max_speed": 7.0,
    "dynamic_max_speed": True,
    "speed_text_enabled": True,
    "speed_text_topic": "/uav1/speed_text",
    "speed_text_height": 0.20,
    "speed_text_offset": [0.30, 0.0, 0.40],  # Local-world/ENU metres.
    "speed_text_lifetime": 0.50,
}
COLOR_STOPS = (
    (0.00, (0.000, 0.000, 0.502)),
    (0.15, (0.118, 0.565, 1.000)),
    (0.30, (0.196, 0.804, 0.196)),
    (0.45, (0.604, 0.804, 0.196)),
    (0.55, (1.000, 0.843, 0.000)),
    (0.70, (1.000, 0.647, 0.000)),
    (0.85, (1.000, 0.000, 0.000)),
    (1.00, (0.545, 0.000, 0.000)),
)
COLOR_ALPHA = 0.8
QUATERNION_EPSILON = 1e-6


class DroneTrajectoryPublisher:
    def __init__(self):
        for name, default in DEFAULT_PARAMETERS.items():
            setattr(self, name, rospy.get_param("~" + name, default))
        self.max_points = int(self.max_points)
        self.line_width = float(self.line_width)
        self.max_speed = float(self.max_speed)
        self.speed_text_height = float(self.speed_text_height)
        self.speed_text_lifetime = float(self.speed_text_lifetime)
        self.speed_text_offset = tuple(float(value) for value in self.speed_text_offset)
        if self.max_points < 2:
            raise ValueError("~max_points must be at least 2")
        if not math.isfinite(self.line_width) or self.line_width <= 0:
            raise ValueError("~line_width must be finite and positive")
        if not math.isfinite(self.max_speed) or self.max_speed <= 0:
            raise ValueError("~max_speed must be finite and positive")
        if not self.world_frame or self.world_frame == self.body_frame:
            raise ValueError("world_frame must be nonempty and differ from body_frame")
        if not self.speed_text_topic:
            raise ValueError("~speed_text_topic must be nonempty")
        if not math.isfinite(self.speed_text_height) or self.speed_text_height <= 0:
            raise ValueError("~speed_text_height must be finite and positive")
        if not math.isfinite(self.speed_text_lifetime) or self.speed_text_lifetime <= 0:
            raise ValueError("~speed_text_lifetime must be finite and positive")
        if len(self.speed_text_offset) != 3 or not all(math.isfinite(v) for v in self.speed_text_offset):
            raise ValueError("~speed_text_offset must contain three finite values")

        self.points = deque(maxlen=self.max_points)
        self.colors = deque(maxlen=self.max_points)
        self.traj_marker = self._create_marker()
        self.traj_pub = rospy.Publisher(
            self.trajectory_topic, Marker, queue_size=1
        )
        self.speed_text_pub = (
            rospy.Publisher(self.speed_text_topic, Marker, queue_size=1)
            if self.speed_text_enabled else None
        )
        self.tf_broadcaster = (
            tf2_ros.TransformBroadcaster() if self.publish_tf else None
        )
        # Subscribe last: all callback state must exist before data arrives.
        self.odom_sub = rospy.Subscriber(
            self.odom_topic, Odometry, self.odom_callback, queue_size=1
        )
        rospy.loginfo(
            "[RVIZ] odom=%s, marker=%s, frame=%s, max_points=%d, TF=%s. "
            "world_frame must share the odometry world origin; no frame conversion is applied.",
            self.odom_topic, self.trajectory_topic, self.world_frame,
            self.max_points, "ON" if self.publish_tf else "OFF",
        )
        rospy.loginfo(
            "[RVIZ] speed_text=%s, height=%.2f m, lifetime=%.2f s.",
            self.speed_text_topic if self.speed_text_enabled else "OFF",
            self.speed_text_height, self.speed_text_lifetime,
        )

    def _create_marker(self):
        marker = Marker()
        marker.header.frame_id = self.world_frame
        marker.ns = "uav_trajectory"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.line_width
        marker.color.a = COLOR_ALPHA
        return marker

    def _publish_speed_text(self, position, stamp, speed, color):
        if self.speed_text_pub is None:
            return
        label = Marker()
        label.header.frame_id = self.world_frame
        label.header.stamp = stamp
        label.ns = "uav_speed"
        label.id = 0
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.orientation.w = 1.0
        label.pose.position.x = position.x + self.speed_text_offset[0]
        label.pose.position.y = position.y + self.speed_text_offset[1]
        label.pose.position.z = position.z + self.speed_text_offset[2]
        label.scale.z = self.speed_text_height
        label.color = color  # Same RGBA as the newest speed-coloured trace point.
        # ASCII text avoids missing Chinese glyphs in RViz's default font.
        label.text = f"Speed: {speed:.2f} m/s"
        label.lifetime = rospy.Duration(self.speed_text_lifetime)
        self.speed_text_pub.publish(label)

    def get_color_from_speed(self, speed):
        ratio = max(0.0, min(speed / self.max_speed, 1.0))
        for (low, low_rgb), (high, high_rgb) in zip(COLOR_STOPS, COLOR_STOPS[1:]):
            if low <= ratio <= high:
                weight = (ratio - low) / (high - low)
                rgb = [a + (b - a) * weight for a, b in zip(low_rgb, high_rgb)]
                return ColorRGBA(rgb[0], rgb[1], rgb[2], COLOR_ALPHA)
        return ColorRGBA(*COLOR_STOPS[-1][1], COLOR_ALPHA)

    def _publish_transform(self, msg, stamp, quaternion):
        child_frame = self.body_frame or msg.child_frame_id or "base_link"
        if child_frame == self.world_frame:
            rospy.logwarn_throttle(2.0, "[RVIZ] Skipping TF: parent and child frames are identical.")
            return
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.world_frame
        transform.child_frame_id = child_frame
        position = msg.pose.pose.position
        transform.transform.translation.x = position.x
        transform.transform.translation.y = position.y
        transform.transform.translation.z = position.z
        rotation = transform.transform.rotation
        rotation.x, rotation.y, rotation.z, rotation.w = quaternion
        self.tf_broadcaster.sendTransform(transform)

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        velocity = msg.twist.twist.linear
        orientation = msg.pose.pose.orientation
        quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
        values = (position.x, position.y, position.z,
                  velocity.x, velocity.y, velocity.z) + quaternion
        if not all(math.isfinite(value) for value in values):
            rospy.logwarn_throttle(2.0, "[RVIZ] Ignoring non-finite odometry.")
            return
        norm = math.hypot(*quaternion)
        if not math.isfinite(norm) or norm < QUATERNION_EPSILON:
            rospy.logwarn_throttle(2.0, "[RVIZ] Ignoring odometry with an invalid quaternion.")
            return
        quaternion = tuple(value / norm for value in quaternion)
        stamp = msg.header.stamp if msg.header.stamp.to_sec() > 0 else rospy.Time.now()

        # Rotation does not change the speed magnitude used for colouring.
        speed = math.hypot(velocity.x, velocity.y, velocity.z)
        if not math.isfinite(speed):
            rospy.logwarn_throttle(2.0, "[RVIZ] Ignoring overflowing odometry speed.")
            return

        if self.tf_broadcaster is not None:
            self._publish_transform(msg, stamp, quaternion)

        if self.dynamic_max_speed and speed > self.max_speed * 0.95:
            self.max_speed = speed * 1.2
        color = self.get_color_from_speed(speed)
        self.points.append(Point(position.x, position.y, position.z))
        self.colors.append(color)
        self.traj_marker.header.stamp = stamp
        self.traj_marker.points = list(self.points)
        self.traj_marker.colors = list(self.colors)
        self.traj_pub.publish(self.traj_marker)
        self._publish_speed_text(position, stamp, speed, color)


def main():
    rospy.init_node("drone_trajectory_publisher")
    try:
        node = DroneTrajectoryPublisher()
    except (TypeError, ValueError) as error:
        rospy.logfatal("[RVIZ] Invalid configuration: %s", error)
        return
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
