#!/usr/bin/env python3

import os
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage, Range
from geometry_msgs.msg import TwistStamped
from cv_bridge import CvBridge, CvBridgeError

class FollowMe:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('follow_me', anonymous=False)

        # Parameters
        self.rate_hz = rospy.get_param('~rate', 10)
        self.scan_speed = rospy.get_param('~scan_speed', 0.3)
        self.forward_speed = rospy.get_param('~forward_speed', 0.2)
        self.stop_distance = rospy.get_param('~stop_distance', 0.3)
        self.k_ang = rospy.get_param('~angular_gain', 1.0)
        self.cam_freq = rospy.get_param('~cam_freq', 2)
        self.debug = rospy.get_param('~debug', False)

        # CV Bridge
        self.bridge = CvBridge()
        # Image buffers
        self.input_camera = [None, None]
        self.new_frame = [False, False]
        self.frame_width = 640
        self.frame_height = 480
        self.x_centre = self.frame_width / 2.0
        self.y_centre = self.frame_height / 2.0

        # State
        self.current_range = float('inf')
        self.tick_counter = 0
        self.circles = []

        # Topic namespace
        robot = os.getenv('MIRO_ROBOT_NAME', '')
        ns = f"/{robot}" if robot else ''

        # Subscribers
        rospy.Subscriber(ns + '/sensors/caml/compressed', CompressedImage, self._caml_cb, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber(ns + '/sensors/camr/compressed', CompressedImage, self._camr_cb, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber(ns + '/sensors/sonar', Range, self._range_cb)

        # Publisher
        self.cmd_pub = rospy.Publisher(ns + '/control/cmd_vel', TwistStamped, queue_size=1)

        # Loop rate
        self.loop_rate = rospy.Rate(self.rate_hz)

    def _caml_cb(self, msg):
        self._image_cb(msg, 0)

    def _camr_cb(self, msg):
        self._image_cb(msg, 1)

    def _image_cb(self, msg, idx):
        try:
            img = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
            # Convert BGR to RGB for processing
            frame = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            h, w, _ = frame.shape
            self.frame_height, self.frame_width = h, w
            self.x_centre = w / 2.0
            self.y_centre = h / 2.0
            self.input_camera[idx] = frame
            self.new_frame[idx] = True
        except CvBridgeError:
            pass

    def _range_cb(self, msg):
        self.current_range = msg.range

    def _detect_ball(self, frame):
        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        # Hough Circle detection
        circles = cv2.HoughCircles(
            gray, cv2.HOUGH_GRADIENT,
            dp=1, minDist=40,
            param1=50, param2=10,
            minRadius=5, maxRadius=50
        )
        if circles is None:
            return None
        circles = np.uint16(np.around(circles))
        # Select largest circle
        max_circle = max(circles[0], key=lambda c: c[2])
        x, y, r = map(float, max_circle)
        # Normalize: x in [0,1], y unused here, r normalized
        x_norm = (x - self.x_centre) / self.frame_width + 0.5
        r_norm = r / self.frame_width
        return [x_norm, r_norm]

    def run(self):
        rospy.loginfo('[follow_me] Node running - searching for target')
        while not rospy.is_shutdown():
            # Vision detection every cam_freq ticks
            if self.tick_counter % self.cam_freq == 0:
                self.circles = []
                for idx in range(2):
                    if self.new_frame[idx]:
                        det = self._detect_ball(self.input_camera[idx])
                        self.new_frame[idx] = False
                        if det:
                            class Circle: pass
                            c = Circle()
                            c.x, c.radius = det
                            self.circles.append(c)

            cmd = TwistStamped()
            # No detection: rotate to scan
            if not self.circles:
                cmd.twist.angular.z = self.scan_speed
            else:
                # Center on largest detection
                best = max(self.circles, key=lambda c: c.radius)
                x_off = best.x - 0.5
                cmd.twist.angular.z = -self.k_ang * x_off
                # Move forward if beyond stop_distance
                if self.current_range > self.stop_distance:
                    cmd.twist.linear.x = self.forward_speed
                else:
                    # Stop at target distance
                    cmd.twist.linear.x = 0.0
                    cmd.twist.angular.z = 0.0
                    rospy.loginfo('[follow_me] Within stop distance - holding position')
            # Publish command
            self.cmd_pub.publish(cmd)

            self.tick_counter += 1
            self.loop_rate.sleep()

if __name__ == '__main__':
    node = FollowMe()
    node.run()