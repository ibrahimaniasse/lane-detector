#!/usr/bin/env python3
"""
control_blind.py
----------------
ROS1 node: executes a position-triggered blind maneuver when the robot
reaches a known (x, y) location on the track -- e.g. the point where a
support beam physically occludes the camera.

While active, this node:
  1. Publishes True on /control/blind_maneuver/active so that
     control_lane.py silences itself (no competing cmd_vel).
  2. Drives /cmd_vel with a fixed (angular_z, linear_x) command for a
     fixed duration to navigate the occluded section open-loop.
  3. Publishes False on /control/blind_maneuver/active when done, letting
     control_lane.py resume vision-based tracking.

The trigger fires ONCE per run (re-entering the zone after completion does
not re-trigger).  All parameters are exposed as ROS params for tuning.

Subscribes:
  /odom   nav_msgs/Odometry   robot position for trigger detection

Publishes:
  /control/blind_maneuver/active   std_msgs/Bool     handoff flag
  /cmd_vel                         geometry_msgs/Twist  drive command during maneuver
"""

import math
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class ControlBlind:
    def __init__(self):
        rospy.init_node("control_blind", anonymous=False)

        # ---- Trigger zone (circle centered on the beam position) ----
        self.trigger_x = rospy.get_param("~maneuver_trigger_x", 1.02)
        self.trigger_y = rospy.get_param("~maneuver_trigger_y", 2.75)
        self.trigger_radius = rospy.get_param("~maneuver_trigger_radius", 0.3)

        # ---- Maneuver command ----
        self.angular_z = rospy.get_param("~maneuver_angular_z", 0.6)
        self.linear_x = rospy.get_param("~maneuver_linear_x", 0.06)
        self.duration = rospy.get_param("~maneuver_duration", 2.0)

        # ---- State ----
        self.maneuver_active = False
        self.maneuver_done = False      # only fire once per run
        self.maneuver_start_time = None

        # ---- Publishers ----
        self.blind_pub = rospy.Publisher(
            "/control/blind_maneuver/active", Bool, queue_size=1, latch=True)
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        # Publish initial False so control_lane.py knows we exist but aren't active
        self.blind_pub.publish(Bool(data=False))

        # ---- Subscriber ----
        self.odom_sub = rospy.Subscriber("/odom", Odometry,
                                         self.odom_callback, queue_size=1)

        rospy.loginfo(
            "control_blind node started  trigger=(%.2f, %.2f) r=%.2f  "
            "cmd=(ang=%.2f, lin=%.2f) dur=%.1fs",
            self.trigger_x, self.trigger_y, self.trigger_radius,
            self.angular_z, self.linear_x, self.duration)

    # ------------------------------------------------------------------
    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if self.maneuver_active:
            elapsed = (rospy.Time.now() - self.maneuver_start_time).to_sec()
            if elapsed >= self.duration:
                # Maneuver complete — hand control back to vision
                rospy.loginfo(
                    "control_blind: maneuver complete (%.1fs), resuming vision", elapsed)
                self.maneuver_active = False
                self.maneuver_done = True
                self.blind_pub.publish(Bool(data=False))
                # Send one zero-velocity to avoid residual drift
                self.publish_cmd(0.0, 0.0)
            else:
                self.publish_cmd(self.angular_z, self.linear_x)
            return

        # Not currently active — check if we should trigger
        if self.maneuver_done:
            return  # already fired once this run

        dist = math.hypot(x - self.trigger_x, y - self.trigger_y)
        if dist <= self.trigger_radius:
            rospy.loginfo(
                "control_blind: entered trigger zone (dist=%.2fm) — starting maneuver", dist)
            self.maneuver_active = True
            self.maneuver_start_time = rospy.Time.now()
            self.blind_pub.publish(Bool(data=True))
            self.publish_cmd(self.angular_z, self.linear_x)

    # ------------------------------------------------------------------
    def publish_cmd(self, angular_z, linear_x):
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_pub.publish(twist)

    def stop(self):
        """Called on node shutdown — ensure we release the handoff flag."""
        self.blind_pub.publish(Bool(data=False))
        self.publish_cmd(0.0, 0.0)


if __name__ == "__main__":
    node = ControlBlind()
    rospy.on_shutdown(node.stop)
    rospy.spin()
