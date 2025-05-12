#!/usr/bin/env python

import os
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage, Range, Imu
from geometry_msgs.msg import Twist, Pose
from geometry_msgs.msg import TwistStamped  # ROS cmd_vel (velocity control) message

import miro_msgs
from miro_msgs.msg import platform_config, platform_sensors, platform_state, platform_mics, platform_control, core_state, core_control, core_config, bridge_config, bridge_stream

import opencv_apps
from opencv_apps.msg import CircleArrayStamped

import math
import numpy
import time
import sys
from miro_constants import miro
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray

from datetime import datetime


"""
emergency_stop.py implements halts the MiRo when told "Stop".
MiRo responds by stopping all motion immediately
"""

class EmergencyStop():

    def __init__(self):

        ## Node rate
        self.rate = rospy.get_param('rate',200)

        # self.pub_platform_control = rospy.Publisher('/miro_emergency',platform_control,queue_size=0)

        rospy.init_node("movement_publisher")
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        self.vel_pub = rospy.Publisher(
            topic_base_name + "/control/cmd_vel", TwistStamped, queue_size=0
        )

        self.pub_platform_control = rospy.Publisher(self.topic_root + "/miro/control", String, self.control_callback)

    def control_callback(self, msg):
        # Check if the message is "stop"
        if msg.data.strip().lower() == "stop":
            self.miro_emergency()
            rospy.loginfo("Received command to stop")

    def miro_emergency(self):

        r = rospy.Rate(self.rate)
        msg_cmd_vel = TwistStamped()
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        self.vel_pub = rospy.Publisher(
            topic_base_name + "/control/cmd_vel", TwistStamped, queue_size=0
        )

        while True:
            try:
                msg_cmd_vel.twist.linear.x = 0
                msg_cmd_vel.twist.linear.y = 0
                msg_cmd_vel.twist.linear.z = 0

                self.vel_pub.publish(msg_cmd_vel)
            except KeyboardInterrupt:
                self.vel_pub.publish(msg_cmd_vel)
                print("KeyboardInterrupt: Stopping the emergency stop action.")               
                break
            r.sleep()

       

if __name__== '__main__':
    rospy.init_node('stop', disable_signals=True)
    stop = EmergencyStop()
    rospy.spin()