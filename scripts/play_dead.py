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
MiRo responds by stopping all motion, and dropping its head. (idea; also close eyes?)
"""

class PlayDead():

    def __init__(self):
        ## Node rate
        self.rate = rospy.get_param('rate',200)
