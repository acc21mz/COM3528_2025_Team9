#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, UInt16MultiArray, Int16MultiArray, Int8
from sensor_msgs.msg import Image, CompressedImage, Range, Imu
from geometry_msgs.msg import Twist, Pose

import miro_msgs
from miro_msgs.msg import platform_config, platform_sensors, platform_state, platform_mics, platform_control, core_state, core_control, core_config, bridge_config, bridge_stream

# import opencv_apps
# from opencv_apps.msg import CircleArrayStamped

import math
import numpy as np
import time
import sys
from miro_constants import miro
import os
from datetime import datetime
import subprocess


"""
speak.py implements the action corresponding to the "speak" command
MiRo responds by playing a bark audio file
"""
BUFFER_STUFF_BYTES = 4000
MAX_STREAM_MSG_SIZE = (4096 - 48)

class Speak():
    def __init__(self):

        ## Node rate
        self.rate = rospy.get_param('rate',200)
        self.pub_sound = rospy.Publisher('/miro/sound/command', Int8, queue_size=1, latch=True)
        self.file = os.path.join(os.path.dirname(__file__), './bark.wav')
        # file = "/home/student/pkgs/mdk-230105/catkin_ws/src/com3528_team09/cogbiogroup9/scripts/bark.wav"
        # load wav
        with open(self.file, 'rb') as f:
            dat = f.read()
    

        # # convert to numpy array
        dat = np.frombuffer(dat, dtype='int16').astype(np.int32)

        # normalise wav
        dat = dat.astype(np.float)
        sc = 32767.0 / np.max(np.abs(dat))
        dat *= sc
        dat = dat.astype(np.int16).tolist()
        self.data = dat

        # robot name
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")

        # publish
        topic = topic_base_name + "/control/stream"
        print ("publish", topic)
        self.pub_stream = rospy.Publisher(topic, Int16MultiArray, queue_size=0)
        # subscribe
        topic = topic_base_name + "/platform/log"
        print ("subscribe", topic)
        self.sub_log = rospy.Subscriber(topic, String, self.callback_log, queue_size=5, tcp_nodelay=True)

        # subscribe
        topic = topic_base_name + "/sensors/stream"
        print ("subscribe", topic)
        self.sub_stream = rospy.Subscriber(topic, UInt16MultiArray, self.callback_stream, queue_size=1, tcp_nodelay=True)#

    def callback_log(self, msg):
        sys.stdout.write(msg.data)
        sys.stdout.flush()

    def callback_stream(self, msg):
        self.buffer_space = msg.data[0]
        self.buffer_total = msg.data[1]

    def bark(self):
        # subprocess.Popen(['mpg123', self.file])
        # sound = Int8()
        # sound.data = self.data
        # self.pub_sound(sound)
        # while not rospy.core.is_shutdown():
        # if we've received a report
        chunk_size = 1024
        for i in range(0, len(self.data), chunk_size):
            msg = Int16MultiArray()
            msg.data = self.data[i:i+chunk_size]
            self.pub_stream.publish(msg)
            rospy.sleep(0.01)

        rospy.sleep(2)
if __name__ == "__main__":

    rospy.init_node("speak", anonymous=True)
    main = Speak()
    main.bark()
