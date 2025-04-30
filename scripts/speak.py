#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, UInt16MultiArray, Int16MultiArray
from sensor_msgs.msg import Image, CompressedImage, Range, Imu
from geometry_msgs.msg import Twist, Pose

import miro_msgs
from miro_msgs.msg import platform_config, platform_sensors, platform_state, platform_mics, platform_control, core_state, core_control, core_config, bridge_config, bridge_stream

import opencv_apps
from opencv_apps.msg import CircleArrayStamped

import math
import numpy as np
import time
import sys
from miro_constants import miro
import os
from datetime import datetime


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
        file = "../bark.wav"
        # load wav
        with open(file, 'rb') as f:
            dat = f.read()
        self.data_r = 0

        # convert to numpy array
        dat = np.fromstring(dat, dtype='int16').astype(np.int32)

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
        self.sub_stream = rospy.Subscriber(topic, UInt16MultiArray, self.callback_stream, queue_size=1, tcp_nodelay=True)

    def callback_log(self, msg):

        sys.stdout.write(msg.data)
        sys.stdout.flush()

    def callback_stream(self, msg):

        self.buffer_space = msg.data[0]
        self.buffer_total = msg.data[1]

    
    def loop(self):
        while not rospy.core.is_shutdown():
            #check state_file
            # if we've received a report
            if self.buffer_total > 0:

                # compute amount to send
                buffer_rem = self.buffer_total - self.buffer_space
                n_bytes = BUFFER_STUFF_BYTES - buffer_rem
                n_bytes = max(n_bytes, 0)
                n_bytes = min(n_bytes, MAX_STREAM_MSG_SIZE)

                # if amount to send is non-zero
                if n_bytes > 0:

                    msg = Int16MultiArray(data = self.data[self.data_r:self.data_r+n_bytes])
                    self.pub_stream.publish(msg)
                    self.data_r += n_bytes

            # break
            if self.data_r >= len(self.data):
                break

            # # report once per second
            # if count == 0:
            #     count = 10
            #     print ("streaming:", self.data_r, "/", len(self.data), "bytes")

            #     # check at those moments if we are making progress, also
            #     if dropout_data_r == self.data_r:
            #         if dropout_count == 0:
            #             print ("dropping out because of no progress...")
            #             break
            #         print ("dropping out in", str(dropout_count) + "...")
            #         dropout_count -= 1
            #     else:
                    # dropout_data_r = self.data_r

            # count tenths
            # count -= 1
            time.sleep(0.1)
if __name__ == "__main__":

    rospy.init_node("client_stream", anonymous=True)
    main = Speak()
    main.loop(sys.argv[2:])
