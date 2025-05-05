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

from datetime import datetime


"""
emergency_stop.py implements halts the MiRo when told "Stop".
MiRo responds by stopping all motion immediately
"""

class EmergencyStop():

    def __init__(self):

        ## Node rate
        self.rate = rospy.get_param('rate',200)

        self.pub_platform_control = rospy.Publisher('/miro_emergency',platform_control,queue_size=0)
    
    ## Function that sets the parameters of the structure platform_control corresponding to action "Stop"
    def miro_emergency(self):

        r = rospy.Rate(self.rate)
        q = platform_control()

        #VELOCITY CODE TAKEN FROM FETCH.PY

        # Prepare an empty velocity command message
        msg_cmd_vel = TwistStamped()
        # Individual robot name acts as ROS topic prefix
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        self.vel_pub = rospy.Publisher(
            topic_base_name + "/control/cmd_vel", TwistStamped, queue_size=0
        )
        
        while True:
            try:
                # q.eyelid_closure = 1.0  # close eyelids
                # q.body_config = [0.0,1.0,0.2,0.1]  # move head down
                # q.body_config_speed = [0.0,-1.0,-1.0,-1.0]    # control speed at which head falls
                # q.tail = -1.0 # bring tail down
                # q.ear_rotate = [1.0,1.0]  # point ears down
                msg_cmd_vel.twist.linear.x = 0
                msg_cmd_vel.twist.linear.y = 0
                msg_cmd_vel.twist.linear.z = 0

                self.vel_pub.publish(msg_cmd_vel)
                # self.pub_platform_control.publish(q)
            except KeyboardInterrupt:
                self.vel_pub.publish(msg_cmd_vel)
                # self.pub_platform_control.publish(q)                
                break
            r.sleep()

       

if __name__== '__main__':
    rospy.init_node('stop', disable_signals=True)
    stop = EmergencyStop()
    stop.miro_emergency()