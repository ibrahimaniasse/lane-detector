#!/usr/bin/env python3
"""
test_ramp_climb.py  (ROS 2 / rclpy)
------------------------------------
Test node to validate ramp physics and friction.
Drives the TurtleBot3 straight forward along +Y up the ramp, monitoring
odometry altitude (Z) and distance traveled.

Usage:
  ros2 run lane_detector test_ramp_climb --ros-args -p use_sim_time:=true
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class TestRampClimb(Node):
    def __init__(self):
        super().__init__('test_ramp_climb')

        self.declare_parameter('speed', 0.2)
        self.declare_parameter('target_z', 0.15)     # Altitude threshold for climbing
        self.declare_parameter('timeout_sec', 60.0)  # 60s timeout

        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)

        self.speed = self.get_parameter('speed').value
        self.target_z = self.get_parameter('target_z').value
        self.timeout_sec = self.get_parameter('timeout_sec').value

        self.start_time = None
        self.start_y = None
        self.current_z = 0.0
        self.max_z = 0.0
        self.odom_received = False
        self.success = False

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(
            f'=== Ramp Climb Test Node Started ===\n'
            f'Target Speed: {self.speed} m/s | Target Height (Z): >= {self.target_z:.2f}m\n'
            f'Waiting for /odom messages to begin driving...')

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        self.current_z = pos.z

        if not self.odom_received:
            self.odom_received = True
            self.start_y = pos.y
            self.start_time = self.get_clock().now()
            self.get_logger().info(
                f'✅ /odom received! Initial Y={self.start_y:.2f}m, Z={pos.z:.3f}m. Starting drive...')

        if pos.z > self.max_z:
            self.max_z = pos.z

        dist_y = pos.y - self.start_y if self.start_y is not None else 0.0
        self.get_logger().info(
            f'Position: Y={pos.y:.2f}m (dist={dist_y:.2f}m) | Height Z={pos.z:.3f}m | Max Z={self.max_z:.3f}m',
            throttle_duration_sec=1.0)

        if pos.z >= self.target_z and not self.success:
            self.success = True
            self.get_logger().info(
                f'🎉 SUCCESS! Robot climbed to top of ramp! Peak altitude Z = {pos.z:.3f}m')

    def control_loop(self):
        if not self.odom_received:
            return  # Wait for first odom message

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        if self.success:
            self.publish_cmd(0.0, 0.0)
            return

        if elapsed > self.timeout_sec:
            self.get_logger().error(
                f'❌ TIMEOUT ({self.timeout_sec}s)! Peak Z reached: {self.max_z:.3f}m (Target: {self.target_z:.2f}m)')
            self.publish_cmd(0.0, 0.0)
            return

        # Drive straight forward
        self.publish_cmd(0.0, self.speed)

    def publish_cmd(self, angular_z, linear_x):
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_pub.publish(twist)

    def stop(self):
        if rclpy.ok():
            try:
                self.publish_cmd(0.0, 0.0)
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = TestRampClimb()
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
