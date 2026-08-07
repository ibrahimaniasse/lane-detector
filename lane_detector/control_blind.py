#!/usr/bin/env python3
"""
control_blind.py  (ROS 2 / rclpy)
-----------------------------------
ROS 2 node: executes a position-triggered blind maneuver when the robot
reaches a known (x, y) location on the track -- e.g. the point where a
support beam physically occludes the camera.

While active, this node:
  1. Publishes True on /control/blind_maneuver/active so that
     control_lane silences itself (no competing cmd_vel).
  2. Drives /cmd_vel with a fixed (angular_z, linear_x) command for a
     fixed duration to navigate the occluded section open-loop.
  3. Publishes False on /control/blind_maneuver/active when done, letting
     control_lane resume vision-based tracking.

The trigger fires ONCE per run. All parameters are ROS params for tuning.

Subscribes:
  /odom   nav_msgs/Odometry   robot position for trigger detection

Publishes:
  /control/blind_maneuver/active   std_msgs/Bool      handoff flag (latched)
  /cmd_vel                         geometry_msgs/Twist during maneuver
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class ControlBlind(Node):
    def __init__(self):
        super().__init__('control_blind')

        self.declare_parameter('maneuver_trigger_x', 1.02)
        self.declare_parameter('maneuver_trigger_y', 2.75)
        self.declare_parameter('maneuver_trigger_radius', 0.3)
        self.declare_parameter('maneuver_angular_z', 0.6)
        self.declare_parameter('maneuver_linear_x', 0.06)
        self.declare_parameter('maneuver_duration', 2.0)

        self.trigger_x = self.get_parameter('maneuver_trigger_x').value
        self.trigger_y = self.get_parameter('maneuver_trigger_y').value
        self.trigger_radius = self.get_parameter('maneuver_trigger_radius').value
        self.angular_z = self.get_parameter('maneuver_angular_z').value
        self.linear_x = self.get_parameter('maneuver_linear_x').value
        self.duration = self.get_parameter('maneuver_duration').value

        self.maneuver_active = False
        self.maneuver_done = False
        self.maneuver_start_time = None

        # Latched publisher: TRANSIENT_LOCAL so control_lane receives
        # the initial False even if it subscribes after we publish it
        latch_qos = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.blind_pub = self.create_publisher(
            Bool, '/control/blind_maneuver/active', latch_qos)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 1)

        # Publish initial False immediately
        self.blind_pub.publish(Bool(data=False))

        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 1)

        self.get_logger().info(
            f'control_blind started  '
            f'trigger=({self.trigger_x:.2f}, {self.trigger_y:.2f}) '
            f'r={self.trigger_radius:.2f}  '
            f'cmd=(ang={self.angular_z:.2f}, lin={self.linear_x:.2f}) '
            f'dur={self.duration:.1f}s')

    # ------------------------------------------------------------------
    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if self.maneuver_active:
            elapsed = (
                self.get_clock().now() - self.maneuver_start_time
            ).nanoseconds / 1e9
            if elapsed >= self.duration:
                self.get_logger().info(
                    f'control_blind: maneuver complete ({elapsed:.1f}s), '
                    f'resuming vision')
                self.maneuver_active = False
                self.maneuver_done = True
                self.blind_pub.publish(Bool(data=False))
                self.publish_cmd(0.0, 0.0)
            else:
                self.publish_cmd(self.angular_z, self.linear_x)
            return

        if self.maneuver_done:
            return

        dist = math.hypot(x - self.trigger_x, y - self.trigger_y)
        if dist <= self.trigger_radius:
            self.get_logger().info(
                f'control_blind: entered trigger zone (dist={dist:.2f}m) '
                f'— starting maneuver')
            self.maneuver_active = True
            self.maneuver_start_time = self.get_clock().now()
            self.blind_pub.publish(Bool(data=True))
            self.publish_cmd(self.angular_z, self.linear_x)

    # ------------------------------------------------------------------
    def publish_cmd(self, angular_z, linear_x):
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_pub.publish(twist)

    def stop(self):
        if rclpy.ok():
            try:
                self.blind_pub.publish(Bool(data=False))
                self.publish_cmd(0.0, 0.0)
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = ControlBlind()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
