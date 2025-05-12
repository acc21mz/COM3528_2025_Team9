#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

class CommandRecognition:
    def __init__(self):
        self.rate = rospy.get_param('rate', 200)
        self.robot_name = rospy.get_param('/robot_name', 'miro')
        self.topic_root = "/miro/" + self.robot_name
        print("topic_root", self.topic_root)

        self.activate = False
        self.command = ""

        # Publisher for control commands as strings
        self.pub_control = rospy.Publisher('/miro/control', String, queue_size=1)
        # Subscriber to speech-to-text
        rospy.Subscriber(self.topic_root + '/speech_to_text', String, self.speech_callback, queue_size=1)

    def speech_callback(self, msg):
        self.command = msg.data.strip().lower()
        print(f"🗣️ Heard: {self.command}")
        if self.command == "miro":
            self.activate = True
        elif self.activate:
            if self.command in ["play dead", "play dad"]:
                self.pub_control.publish("play_dead")
                print("Play dead")
            elif self.command == "stop":
                self.pub_control.publish("stop")
                print("Stop")
            elif self.command == "fetch":
                self.pub_control.publish("fetch")
                print("fetch")
            elif self.command == "follow me":
                self.pub_control.publish("follow_me")
                print("follow me")
            elif self.command == "speak":
                self.pub_control.publish("speak")
                print("speak")
            else:
                print(f"Unrecognised command: {self.command}")
                self.pub_control.publish("unrecognised:" + self.command)
            self.activate = False

    def switching_commands(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            r.sleep()


if __name__== '__main__':
    rospy.init_node('command_recognition')
    sb = CommandRecognition()
    sb.switching_commands()