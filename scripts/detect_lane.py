#!/usr/bin/env python3
"""
detect_lane.py
--------------
ROS1 node: detects the YELLOW (left) and WHITE (right) lane lines using a
bird's-eye (perspective-warped) view, finds each lane's pixels with a
sliding-window search, fits a 2nd-order polynomial, smooths it over several
frames, and publishes the normalized lane center offset for control_lane.py.

Simplified to closely match ROBOTIS's own turtlebot3_autorace detect_lane
approach:
  - Auto-adjusting brightness (lightness/value) floor per lane color,
    self-tuning every frame based on how much got detected -- adapts to
    shadow without needing a fixed threshold.
  - A reliability score (0-100) per lane based on row-coverage, rather than
    a hard binary detected/not-detected.
  - When only one lane is reliable, fall back to that lane's position plus
    a fixed lane-width offset (like ROBOTIS's "centerx +/- 320").
  - When NEITHER lane is reliable, simply don't publish anything this frame
    (matching ROBOTIS's is_center_x_exist=False behavior) rather than
    publishing NaN -- the robot naturally continues on its last valid
    command rather than needing special "lost lane" handling downstream.

Subscribes:
  ~image_topic (default /camera/image_raw)          sensor_msgs/Image

Publishes:
  /detect/lane_center       std_msgs/Float64   normalized offset in [-1, 1]
                                                (0 = centered, +1 = lane far right,
                                                 -1 = lane far left)
                                                NOT published at all if neither
                                                lane is currently reliable.
  /detect/image_lane        sensor_msgs/Image  debug: warped view + masks + fitted curves
  /detect/image_warped      sensor_msgs/Image  debug: raw perspective-warp output
"""

import rospy
import cv2
import numpy as np
from collections import deque
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from std_msgs.msg import Float64, UInt8


class DetectLane:
    def __init__(self):
        rospy.init_node("detect_lane", anonymous=False)

        # ---------------- Parameters ----------------
        self.image_topic = rospy.get_param("~image_topic", "/camera/image_raw")

        self.warp_src = np.float32([
            rospy.get_param("~warp_src_tl", [80, 140]),
            rospy.get_param("~warp_src_tr", [240, 140]),
            rospy.get_param("~warp_src_br", [319, 239]),
            rospy.get_param("~warp_src_bl", [0, 239]),
        ])
        self.warp_w = rospy.get_param("~warp_width", 320)
        self.warp_h = rospy.get_param("~warp_height", 300)
        margin = rospy.get_param("~warp_dst_margin", 100)
        self.warp_dst = np.float32([
            [margin, 0],
            [self.warp_w - margin, 0],
            [self.warp_w - margin, self.warp_h],
            [margin, self.warp_h],
        ])
        self.M = cv2.getPerspectiveTransform(self.warp_src, self.warp_dst)
        self.yellow_reliability_pub = rospy.Publisher("/detect/yellow_line_reliability", UInt8, queue_size=1)
        self.white_reliability_pub  = rospy.Publisher("/detect/white_line_reliability",  UInt8, queue_size=1)

        self.yellow_lower = np.array(rospy.get_param("~yellow_lower", [15, 50, 50]), dtype=np.float64)
        self.yellow_upper = np.array(rospy.get_param("~yellow_upper", [35, 255, 255]), dtype=np.float64)
        self.white_lower  = np.array(rospy.get_param("~white_lower",  [0, 0, 180]), dtype=np.float64)
        self.white_upper  = np.array(rospy.get_param("~white_upper",  [180, 60, 255]), dtype=np.float64)

        self.measured_lane_width = rospy.get_param("~assumed_lane_width_px", 78)
        self.mov_avg_length = rospy.get_param("~mov_avg_length", 5)
        self.min_pixels = rospy.get_param("~min_lane_pixels", 50)
        self.max_pixels = rospy.get_param("~max_lane_pixels", 3000)
        self.lookahead_frac = rospy.get_param("~lookahead_frac", 0.9)

        self.num_windows = rospy.get_param("~num_windows", 20)  # matches ROBOTIS
        self.window_margin = rospy.get_param("~window_margin", 40)
        self.window_minpix = rospy.get_param("~window_minpix", 15)

        self.mov_avg_left = deque(maxlen=self.mov_avg_length)
        self.mov_avg_right = deque(maxlen=self.mov_avg_length)

        # ROBOTIS-style reliability score (0-100) per lane
        self.reliability_yellow = 100
        self.reliability_white = 100
        self.reliability_step = rospy.get_param("~reliability_step", 5)
        self.reliability_threshold = rospy.get_param("~reliability_threshold", 50)
        self.row_coverage_threshold = rospy.get_param("~row_coverage_threshold", 0.7)

        # ROBOTIS-style auto-adjusting brightness floor
        self.auto_adjust_lightness = rospy.get_param("~auto_adjust_lightness", True)
        self.lightness_step = rospy.get_param("~lightness_step", 5)
        self.yellow_v_floor_min = rospy.get_param("~yellow_v_floor_min", 20)
        self.yellow_v_floor_max = rospy.get_param("~yellow_v_floor_max", 250)
        self.white_v_floor_min = rospy.get_param("~white_v_floor_min", 100)
        self.white_v_floor_max = rospy.get_param("~white_v_floor_max", 250)
        self.mask_pixels_high = rospy.get_param("~mask_pixels_high", 8000)
        self.mask_pixels_low = rospy.get_param("~mask_pixels_low", 400)

        clahe_clip_limit = rospy.get_param("~clahe_clip_limit", 2.0)
        clahe_tile_size = rospy.get_param("~clahe_tile_size", 8)
        self.clahe = cv2.createCLAHE(clipLimit=clahe_clip_limit, tileGridSize=(clahe_tile_size, clahe_tile_size))

        # ---------------- ROS plumbing ----------------
        self.bridge = CvBridge()
        self.center_pub = rospy.Publisher("/detect/lane_center", Float64, queue_size=1)
        self.debug_pub  = rospy.Publisher("/detect/image_lane", Image, queue_size=1)
        self.warp_pub   = rospy.Publisher("/detect/image_warped", Image, queue_size=1)
        self.image_sub  = rospy.Subscriber(self.image_topic, Image, self.image_callback,
                                            queue_size=1, buff_size=2**24)

        rospy.loginfo("detect_lane node started, listening on %s", self.image_topic)

    # ------------------------------------------------------------------
    def warp(self, frame):
        return cv2.warpPerspective(frame, self.M, (self.warp_w, self.warp_h))

    def find_base_x(self, mask, side):
        histogram = np.sum(mask[int(self.warp_h * 2 / 3):, :], axis=0)
        half = self.warp_w // 2
        if side == "left":
            region = histogram[:half]
            if region.max() > 0:
                return int(np.argmax(region))
            return self.warp_w // 4
        else:
            region = histogram[half:]
            if region.max() > 0:
                return half + int(np.argmax(region))
            return int(self.warp_w * 3 / 4)

    def sliding_window_fit(self, mask, mov_avg_buffer, side):
        nonzero_y, nonzero_x = mask.nonzero()
        if len(nonzero_x) < self.min_pixels or len(nonzero_x) > self.max_pixels:
            return np.mean(np.array(mov_avg_buffer), axis=0) if len(mov_avg_buffer) > 0 else None

        x_current = self.find_base_x(mask, side)
        window_height = self.warp_h // self.num_windows

        lane_inds = []
        for window in range(self.num_windows):
            y_low = self.warp_h - (window + 1) * window_height
            y_high = self.warp_h - window * window_height
            x_low = x_current - self.window_margin
            x_high = x_current + self.window_margin

            good = ((nonzero_y >= y_low) & (nonzero_y < y_high) &
                    (nonzero_x >= x_low) & (nonzero_x < x_high)).nonzero()[0]
            lane_inds.append(good)
            if len(good) > self.window_minpix:
                x_current = int(np.mean(nonzero_x[good]))

        lane_inds = np.concatenate(lane_inds) if len(lane_inds) else np.array([], dtype=int)
        if len(lane_inds) < self.min_pixels:
            return np.mean(np.array(mov_avg_buffer), axis=0) if len(mov_avg_buffer) > 0 else None

        lane_x = nonzero_x[lane_inds]
        lane_y = nonzero_y[lane_inds]
        coeffs = np.polyfit(lane_y, lane_x, 2)
        mov_avg_buffer.append(coeffs)
        return np.mean(np.array(mov_avg_buffer), axis=0)

    @staticmethod
    def x_at_row(coeffs, y):
        a, b, c = coeffs
        return a * y**2 + b * y + c

    def update_reliability(self, mask, current_reliability):
        rows_with_pixels = np.count_nonzero(mask.any(axis=1))
        rows_missing = mask.shape[0] - rows_with_pixels
        if rows_missing > mask.shape[0] * self.row_coverage_threshold:
            return max(0, current_reliability - self.reliability_step)
        else:
            return min(100, current_reliability + self.reliability_step)

    def auto_adjust_v_floor(self, lower_arr, pixel_count, v_min, v_max):
        v_floor = lower_arr[2]
        if pixel_count > self.mask_pixels_high and v_floor < v_max:
            v_floor = min(v_max, v_floor + self.lightness_step)
        elif pixel_count < self.mask_pixels_low and v_floor > v_min:
            v_floor = max(v_min, v_floor - self.lightness_step)
        lower_arr[2] = v_floor
        return lower_arr

    # ------------------------------------------------------------------
    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr("cv_bridge error: %s", e)
            return

        warped = self.warp(frame)
        hsv = cv2.cvtColor(warped, cv2.COLOR_BGR2HSV)

        h_ch, s_ch, v_ch = cv2.split(hsv)
        v_eq = self.clahe.apply(v_ch)
        hsv = cv2.merge([h_ch, s_ch, v_eq])

        yellow_mask = cv2.inRange(hsv, self.yellow_lower, self.yellow_upper)
        white_mask  = cv2.inRange(hsv, self.white_lower,  self.white_upper)
        white_mask = cv2.bitwise_and(white_mask, cv2.bitwise_not(yellow_mask))

        kernel = np.ones((5, 5), np.uint8)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel)
        white_mask  = cv2.morphologyEx(white_mask,  cv2.MORPH_OPEN, kernel)

        yellow_pixel_count = np.count_nonzero(yellow_mask)
        white_pixel_count = np.count_nonzero(white_mask)

        if self.auto_adjust_lightness:
            self.yellow_lower = self.auto_adjust_v_floor(
                self.yellow_lower, yellow_pixel_count, self.yellow_v_floor_min, self.yellow_v_floor_max)
            self.white_lower = self.auto_adjust_v_floor(
                self.white_lower, white_pixel_count, self.white_v_floor_min, self.white_v_floor_max)

        self.reliability_yellow = self.update_reliability(yellow_mask, self.reliability_yellow)
        self.reliability_white = self.update_reliability(white_mask, self.reliability_white)
        self.yellow_reliability_pub.publish(UInt8(int(self.reliability_yellow)))
        self.white_reliability_pub.publish(UInt8(int(self.reliability_white)))

        left_fit  = self.sliding_window_fit(yellow_mask, self.mov_avg_left, "left")
        right_fit = self.sliding_window_fit(white_mask,  self.mov_avg_right, "right")

        lookahead_y = int(self.warp_h * self.lookahead_frac)

        left_x  = self.x_at_row(left_fit,  lookahead_y) if left_fit  is not None else None
        right_x = self.x_at_row(right_fit, lookahead_y) if right_fit is not None else None

        yellow_ok = self.reliability_yellow > self.reliability_threshold and left_x is not None
        white_ok  = self.reliability_white  > self.reliability_threshold and right_x is not None

        if yellow_ok and white_ok:
            self.measured_lane_width = 0.8 * self.measured_lane_width + 0.2 * abs(right_x - left_x)
            center_x = (left_x + right_x) / 2.0
        elif yellow_ok:
            center_x = left_x + self.measured_lane_width / 2.0
        elif white_ok:
            center_x = right_x - self.measured_lane_width / 2.0
        else:
            # Neither lane reliable -- ROBOTIS's is_center_x_exist=False
            # equivalent: don't publish anything this frame. The robot
            # simply continues on its last valid command rather than
            # needing special "lost lane" handling.
            rospy.logwarn_throttle(2.0, "detect_lane: neither lane reliable, not publishing")
            self.publish_debug(warped, yellow_mask, white_mask, left_x, right_x, None, lookahead_y)
            return

        image_center_x = self.warp_w / 2.0
        normalized_offset = (center_x - image_center_x) / image_center_x
        normalized_offset = max(-1.5, min(1.5, normalized_offset))

        self.center_pub.publish(Float64(normalized_offset))
        self.publish_debug(warped, yellow_mask, white_mask, left_x, right_x, center_x, lookahead_y)

    # ------------------------------------------------------------------
    def publish_debug(self, warped, yellow_mask, white_mask, left_x, right_x, center_x, y_draw):
        if self.debug_pub.get_num_connections() == 0 and self.warp_pub.get_num_connections() == 0:
            return

        try:
            self.warp_pub.publish(self.bridge.cv2_to_imgmsg(warped, encoding="bgr8"))
        except CvBridgeError as e:
            rospy.logerr("cv_bridge error (warp publish): %s", e)

        debug_img = warped.copy()
        debug_img[yellow_mask > 0] = [0, 255, 255]
        debug_img[white_mask > 0]  = [255, 255, 255]

        if left_x is not None:
            cv2.circle(debug_img, (int(left_x), y_draw), 6, (0, 165, 255), -1)
        if right_x is not None:
            cv2.circle(debug_img, (int(right_x), y_draw), 6, (128, 128, 128), -1)
        if center_x is not None:
            cv2.circle(debug_img, (int(center_x), y_draw), 6, (0, 0, 255), -1)
            cv2.line(debug_img, (int(center_x), 0), (int(center_x), self.warp_h), (0, 0, 255), 1)

        try:
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug_img, encoding="bgr8"))
        except CvBridgeError as e:
            rospy.logerr("cv_bridge error (debug publish): %s", e)


if __name__ == "__main__":
    node = DetectLane()
    rospy.spin()