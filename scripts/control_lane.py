#!/usr/bin/env python3
"""
control_lane.py
---------------
ROS1 node: subscribes to the normalized lane-center offset published by
detect_lane.py and runs a PD controller to steer the robot, publishing
geometry_msgs/Twist to /cmd_vel.

Simplified to closely match ROBOTIS's own turtlebot3_autorace control_lane:
a plain PD controller with power-law speed reduction in corners. No special
"lost lane" handling -- since detect_lane.py simply stops publishing when
neither lane is reliable (rather than publishing NaN), this node's callback
just doesn't fire during a brief dropout, and the robot naturally continues
on its last commanded velocity until vision reacquires the lane. A watchdog
still forces a full stop if the lane stays lost for a genuinely long time,
as a safety net.

Also listens on /control/blind_maneuver/active (std_msgs/Bool). While True,
this node goes silent (no cmd_vel published, watchdog disabled) so that a
separate control_blind.py node can own /cmd_vel for a known blind maneuver
(e.g. the beam-occlusion curve). On the falling edge of that flag, the
watchdog's stale timer is reset so it doesn't immediately fire on resume.

Subscribes:
  /detect/lane_center               std_msgs/Float64  normalized offset in [-1, 1]
  /control/blind_maneuver/active    std_msgs/Bool     True while a blind maneuver owns cmd_vel

Publishes:
  ~cmd_vel_topic (default /cmd_vel)   geometry_msgs/Twist
"""

import rospy
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Twist


class ControlLane:
    def __init__(self):
        rospy.init_node("control_lane", anonymous=False)

        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.linear_speed = rospy.get_param("~linear_speed", 0.2)
        self.kp = rospy.get_param("~kp", 1.0)
        self.kd = rospy.get_param("~kd", 0.5)
        self.max_ang_vel = rospy.get_param("~max_ang_vel", 1.5)
        self.min_speed_fraction = rospy.get_param("~min_speed_fraction", 0.3)
        self.speed_falloff_exponent = rospy.get_param("~speed_falloff_exponent", 2.2)  # ROBOTIS uses 2.2
        self.lost_lane_timeout = rospy.get_param("~lost_lane_timeout", 4.0)  # full stop safety net

        self.last_error = 0.0
        self.last_valid_time = rospy.Time.now()

        self.blind_active = False

        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)
        self.center_sub = rospy.Subscriber("/detect/lane_center", Float64, self.center_callback, queue_size=1)
        self.blind_sub = rospy.Subscriber("/control/blind_maneuver/active", Bool,
                                           self.blind_active_callback, queue_size=1)

        self.timer = rospy.Timer(rospy.Duration(0.2), self.watchdog)

        rospy.loginfo("control_lane node started")

    # ------------------------------------------------------------------
    def center_callback(self, msg):
        if self.blind_active:
            return

        error = msg.data
        self.last_valid_time = rospy.Time.now()

        derivative = error - self.last_error
        self.last_error = error

        angular_z = -(self.kp * error + self.kd * derivative)
        angular_z = max(-self.max_ang_vel, min(self.max_ang_vel, angular_z))

        # Power-law speed reduction (ROBOTIS-style): the exponent makes speed
        # drop off much more sharply as error grows than a linear formula
        # would, giving more effective slowdown through real corners.
        speed_factor = max(0.0, 1.0 - min(abs(error), 1.0)) ** self.speed_falloff_exponent
        linear_x = self.linear_speed * max(self.min_speed_fraction, speed_factor)

        self.publish_cmd(angular_z, linear_x)

    def blind_active_callback(self, msg):
        was_active = self.blind_active
        self.blind_active = msg.data
        if was_active and not self.blind_active:
            # Falling edge: blind maneuver just finished. Reset the watchdog
            # timer so it doesn't see a stale last_valid_time (from before
            # the maneuver started) and immediately force-stop on resume.
            self.last_valid_time = rospy.Time.now()

    def watchdog(self, event):
        if self.blind_active:
            return
        elapsed = (rospy.Time.now() - self.last_valid_time).to_sec()
        if elapsed > self.lost_lane_timeout:
            rospy.logwarn_throttle(2.0, "control_lane: no lane_center message for %.1fs, stopping", elapsed)
            self.publish_cmd(0.0, 0.0)

    def publish_cmd(self, angular_z, linear_x):
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_pub.publish(twist)

    def stop(self):
        self.publish_cmd(0.0, 0.0)


if __name__ == "__main__":
    node = ControlLane()
    rospy.on_shutdown(node.stop)
    rospy.spin()