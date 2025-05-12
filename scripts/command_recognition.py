#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import os
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from math import radians
class CommandRecognition:
    def __init__(self):
        self.rate = rospy.get_param('rate', 200)
        self.robot_name = os.getenv("MIRO_ROBOT_NAME", "miro")
        self.topic_root = "/miro/" + self.robot_name
        print("topic_root", self.topic_root)

        self.activate = False
        self.command = ""

        # Publisher for control commands as strings
        self.pub_control = rospy.Publisher('/miro/control', String, queue_size=1)
        self.pub_kinematic = rospy.Publisher(f'/{self.robot_name}/control/kinematic_joints', JointState, queue_size=0)
        self.pub_cosmetic = rospy.Publisher(f'/{self.robot_name}/control/cosmetic_joints', Float32MultiArray, queue_size=0)
        # Subscriber to speech-to-text
        rospy.Subscriber(self.topic_root + '/speech_to_text', String, self.speech_callback, queue_size=1)
        self.kin_joints = JointState()  # Prepare the empty message
        self.cos_joints = Float32MultiArray()  # Prepare the empty message
        self.kin_joints.name = ["tilt", "lift", "yaw", "pitch"]
        self.kin_joints.position = [0.0, radians(0.0), 0.0, 0.0]
        self.cos_joints.data = [0.0, 0.0, 0.0, 0.0, -0.0, -0.0]
        
    def speech_callback(self, msg):
        self.command = msg.data.strip().lower()
        print(f"🗣️ Heard: {self.command}")
        if "miro" in self.command or "myra" in self.command or "mero" in self.command:
            self.activate = True
            self.pub_kinematic.publish([0.0, 0.0, 0.0, 0.0])
            self.pub_cosmetic.publish([0.0, 0.0, 0.0, 0.0, 0.0,0.0])
            if self.activate:
                if "play dead" in self.command or "play dad" in self.command:
                    self.pub_control.publish("play dead")
                    print("Play dead")
                elif "stop" in self.command:
                    self.pub_control.publish("stop")
                    print("Stop")
                elif "fetch" in self.command:
                    self.pub_control.publish("fetch")
                    print("fetch")
                elif "follow me" in self.command:
                    self.pub_control.publish("follow me")
                    print("follow me")
                elif "speak" in self.command:
                    self.pub_control.publish("speak")
                    print("speak")
                elif self.command != "miro" and self.command != "myra":
                    print(f"Unrecognised command: {self.command}")
                    self.pub_control.publish("unrecognised:" + self.command)
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