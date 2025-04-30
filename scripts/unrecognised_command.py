#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Int8

class UnrecognisedCommand:
    def __init__(self):
        self.rate = rospy.get_param('rate', 10)
        self.pub_speak = rospy.Publisher('/tts', String, queue_size=1, latch=True)
        self.pub_platform_control = rospy.Publisher('/miro/platform/control', String, queue_size=1, latch=True)
        self.pub_sound = rospy.Publisher('/miro/sound/command', Int8, queue_size=1, latch=True)

    def unrecognised_response(self):
        r = rospy.Rate(self.rate)

        msg = String()
        msg.data = "I don't understand"
        self.pub_speak.publish(msg)

        q = String()
        q.data = "Unrecognised command response."
        self.pub_platform_control.publish(q)

        sound_msg = Int8()
        sound_msg.data = 5 # beep noise
        self.pub_sound.publish(sound_msg)

        rospy.loginfo("Published unrecognised command response and sound.")
        rospy.sleep(2)

if __name__ == '__main__':
    rospy.init_node('unrecognised_command')
    unrecognised = UnrecognisedCommand()
    unrecognised.unrecognised_response()