#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Int8
import subprocess
import os

class UnrecognisedCommand:
    def __init__(self):
        self.rate = rospy.get_param('rate', 10)
        self.pub_speak = rospy.Publisher('/tts', String, queue_size=1, latch=True)
        self.pub_platform_control = rospy.Publisher('/miro/platform/control', String, queue_size=1, latch=True)
        self.pub_sound = rospy.Publisher('/miro/sound/command', Int8, queue_size=1, latch=True)
        self.beep_path = os.path.join(os.path.dirname(__file__), '../beep-warning-6387.mp3')
        rospy.Subscriber('/unrecognised_command_trigger', String, self.trigger_callback)

    def trigger_callback(self, msg):
        self.unrecognised_response()

    def unrecognised_response(self):
        msg = String()
        msg.data = "I don't understand"
        self.pub_speak.publish(msg)

        q = String()
        q.data = "Unrecognised command response."
        self.pub_platform_control.publish(q)

        # Play the custom beep sound file
        subprocess.Popen(['mpg123', self.beep_path])

        rospy.loginfo("Published unrecognised command response and sound.")
        rospy.sleep(2)

if __name__ == '__main__':
    rospy.init_node('unrecognised_command')
    unrecognised = UnrecognisedCommand()
    rospy.spin()