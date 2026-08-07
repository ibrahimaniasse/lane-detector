#!/usr/bin/env python3
"""
control_lane.py  (ROS 2 / rclpy)
----------------------------------
ROS 2 node: subscribes to the normalized lane-center offset published by
detect_lane and runs a PD controller to steer the robot, publishing
geometry_msgs/Twist to /cmd_vel.

Plain PD controller with power-law speed reduction in corners (ROBOTIS-style).
No special "lost lane" handling -- since detect_lane simply stops publishing
when neither lane is reliable, this node's callback just doesn't fire during
a brief dropout, and the robot naturally continues on its last commanded
velocity until vision reacquires the lane. A watchdog still forces a full
stop if the lane stays lost for a genuinely long time, as a safety net.

Also listens on /control/blind_maneuver/active (std_msgs/Bool). While True,
this node goes silent so that control_blind can own /cmd_vel during the
beam-occlusion section. On the falling edge of that flag, the watchdog's
stale timer is reset so it doesn't immediately fire on resume.

Subscribes:
  /detect/lane_center              std_msgs/Float64  normalized offset [-1, 1]
  /control/blind_maneuver/active   std_msgs/Bool     True while blind maneuver owns cmd_vel

Publishes:
  /cmd_vel   geometry_msgs/Twist
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Twist


class ControlLane(Node):
    def __init__(self):
        super().__init__('control_lane')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('kd', 0.5)
        self.declare_parameter('max_ang_vel', 1.5)
        self.declare_parameter('min_speed_fraction', 0.3)
        self.declare_parameter('speed_falloff_exponent', 2.2)
        self.declare_parameter('lost_lane_timeout', 4.0)

        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.kp = self.get_parameter('kp').value
        self.kd = self.get_parameter('kd').value
        self.max_ang_vel = self.get_parameter('max_ang_vel').value
        self.min_speed_fraction = self.get_parameter('min_speed_fraction').value
        self.speed_falloff_exponent = self.get_parameter('speed_falloff_exponent').value
        self.lost_lane_timeout = self.get_parameter('lost_lane_timeout').value

        self.last_error = 0.0
        self.last_valid_time = self.get_clock().now()
        self.blind_active = False

        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 1)

        self.center_sub = self.create_subscription(
            Float64, '/detect/lane_center', self.center_callback, 1)

        # Use transient local QoS to receive the latched initial False
        # published by control_blind at startup
        latch_qos = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.blind_sub = self.create_subscription(
            Bool, '/control/blind_maneuver/active',
            self.blind_active_callback, latch_qos)

        self.timer = self.create_timer(0.2, self.watchdog)

        self.get_logger().info('control_lane node started')

    # ------------------------------------------------------------------
    def center_callback(self, msg):
        if self.blind_active:
            return

        error = msg.data
        self.last_valid_time = self.get_clock().now()

        derivative = error - self.last_error
        self.last_error = error

        angular_z = -(self.kp * error + self.kd * derivative)
        angular_z = max(-self.max_ang_vel, min(self.max_ang_vel, angular_z))

        # Power-law speed reduction (ROBOTIS-style)
        speed_factor = max(0.0, 1.0 - min(abs(error), 1.0)) ** self.speed_falloff_exponent
        linear_x = self.linear_speed * max(self.min_speed_fraction, speed_factor)

        self.publish_cmd(angular_z, linear_x)

    def blind_active_callback(self, msg):
        was_active = self.blind_active
        self.blind_active = msg.data
        if was_active and not self.blind_active:
            # Falling edge: reset watchdog so it doesn't fire immediately on resume
            self.last_valid_time = self.get_clock().now()

    def watchdog(self):
        if self.blind_active:
            return
        elapsed = (self.get_clock().now() - self.last_valid_time).nanoseconds / 1e9
        if elapsed > self.lost_lane_timeout:
            self.get_logger().warning(
                f'control_lane: no lane_center for {elapsed:.1f}s, stopping',
                throttle_duration_sec=2.0)
            self.publish_cmd(0.0, 0.0)

    def publish_cmd(self, angular_z, linear_x):
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_pub.publish(twist)

    def stop(self):
        self.publish_cmd(0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = ControlLane()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
