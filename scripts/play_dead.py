#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage, Range, Imu
from geometry_msgs.msg import Twist, Pose

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
play_dead.py implements the action corresponding to the "play dead" command
MiRo responds by stopping all motion, dropping its head, closing its eyes, and dro
"""

class PlayDead():

    def __init__(self):

        ## Node rate
        self.rate = rospy.get_param('rate',200)

        self.pub_platform_control = rospy.Publisher('/miro_sleep',platform_control,queue_size=0)
    
    ## Function that sets the parameters of the structure platform_control corresponding to action "Sleep"
    def miro_dead(self):

        r = rospy.Rate(self.rate)
        q = platform_control()
        
        while True:
            try:
                q.eyelid_closure = 1.0  # close eyelids
                q.body_config = [0.0,1.0,0.2,0.1]  # move head down
                q.body_config_speed = [0.0,-1.0,-1.0,-1.0]    # control speed at which head falls
                q.tail = -1.0 # bring tail down
                q.ear_rotate = [1.0,1.0]  # point ears down
                self.pub_platform_control.publish(q)
            except KeyboardInterrupt:
                self.pub_platform_control.publish(q)                
                break
            r.sleep()

       

if __name__== '__main__':
    rospy.init_node('sleep', disable_signals=True)
    sleep = PlayDead()
    sleep.miro_dead()