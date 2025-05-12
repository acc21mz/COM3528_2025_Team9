#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, UInt16MultiArray, Int16MultiArray, Int8
from sensor_msgs.msg import Image, CompressedImage, Range, Imu
from geometry_msgs.msg import Twist, Pose
import math
import numpy as np
import time
import sys
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
        self.robot_name = os.getenv('MIRO_ROBOT_NAME', 'miro')
        self.stream_topic = f'/{self.robot_name}/control/stream'
        self.bark_pcm_path = os.path.join(os.path.dirname(__file__), '../bark.wav')
        self.pub_stream = rospy.Publisher(self.stream_topic, Int16MultiArray, queue_size=1)
        rospy.Subscriber('/miro/control', String, self.control_callback)
        print("Speak node started, listening for 'speak' on /miro/control")

    def control_callback(self, msg):
        print(f"Received on /miro/control: '{msg.data}'")
        if msg.data.strip().lower() == "speak":
            self.play_bark()

    def play_bark(self):
        print("Streaming bark sound to:", self.stream_topic)
        if not os.path.isfile(self.bark_pcm_path):
            print(f"Bark PCM file not found: {self.bark_pcm_path}")
            return
        with open(self.bark_pcm_path, 'rb') as f:
            data = np.frombuffer(f.read(), dtype=np.int16)
        chunk_size = 1024
        for i in range(0, len(data), chunk_size):
            msg = Int16MultiArray()
            msg.data = data[i:i+chunk_size].tolist()
            self.pub_stream.publish(msg)
            rospy.sleep(0.01)
        rospy.loginfo("Published speak command response and streamed bark sound.")
        rospy.sleep(2)

if __name__ == '__main__':
    rospy.init_node('speak_command')
    sc = Speak()
    rospy.spin()
