#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Int16MultiArray
import os
import numpy as np

class UnrecognisedCommand:
    def __init__(self):
        self.rate = rospy.get_param('rate', 10)
        self.robot_name = os.getenv('MIRO_ROBOT_NAME', 'miro')
        self.stream_topic = f'/{self.robot_name}/control/stream'
        self.beep_pcm_path = os.path.join(os.path.dirname(__file__), '../beep-warning-6387.wav')
        self.pub_stream = rospy.Publisher(self.stream_topic, Int16MultiArray, queue_size=1)
        self.pub_speak = rospy.Publisher('/tts', String, queue_size=1, latch=True)
        rospy.Subscriber('/miro/control', String, self.control_callback)

    def control_callback(self, msg):
        if msg.data.startswith('unrecognised:'):
            self.unrecognised_response()

    def unrecognised_response(self):
        msg = String()
        msg.data = "I don't understand"
        self.pub_speak.publish(msg)

        print("Streaming sound to:", self.stream_topic)
        if not os.path.isfile(self.beep_pcm_path):
            print(f"PCM file not found: {self.beep_pcm_path}")
            return
        with open(self.beep_pcm_path, 'rb') as f:
            data = np.frombuffer(f.read(), dtype=np.int16)
        chunk_size = 1024
        for i in range(0, len(data), chunk_size):
            msg = Int16MultiArray()
            msg.data = data[i:i+chunk_size]
            self.pub_stream.publish(msg)
            rospy.sleep(0.01)

        rospy.loginfo("Published unrecognised command response and streamed sound.")
        rospy.sleep(2)

if __name__ == '__main__':
    rospy.init_node('miro_unrecognised_command')
    unrecognised = UnrecognisedCommand()
    rospy.spin()