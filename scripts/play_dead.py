#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage, Range, Imu
from std_msgs.msg import FloatMultiArray
from geometry_msgs.msg import Twist, Pose
from datetime import datetime
from sensor_msgs.msg import JointState
import os
from math import radians  

# play dead makes miro close its eyes and point its ears and tail down.

class PlayDead():

    def __init__(self):

        ## Node rate
        self.rate = rospy.get_param('rate',200)
        self.robot_name = os.getenv('MIRO_ROBOT_NAME', 'miro')
        self.kinematic_control_topic = f'/{self.robot_name}/sensors/kinematic_joints'
        self.pub_kinematic = rospy.Publisher(self.kinematic_control_topic, JointState, queue_size=1)
        self.pub_cosmetic = rospy.Publisher('/miro/control/cosmetic_joints', FloatMultiArray, queue_size=1)
        rospy.Subscriber('/miro/control',String, self.control_callback)
        
    def control_callback(self, msg):
        if 'play dead' in msg.data:
            self.miro_dead()
            print("Received command to play dead")
    
    def miro_dead(self):

        r = rospy.Rate(self.rate)
        self.kin_joints = JointState()  # Prepare the empty message
        self.cos_joints = FloatMultiArray()  # Prepare the empty message
        try:
            self.kin_joints.name = ["tilt", "lift", "yaw", "pitch"]
            self.kin_joints.position = [0.0, radians(0), 0.0, 0.0]
            self.cos_joints.name = ["droop", "wag", "eyel", "eyer", "earl", "earr"]
            self.cos_joints.data = [1.0, 0.0, 0.0, 0.0, -1.0, -1.0]
            self.pub_kinematic.publish(self.kin_joints)
            self.pub_cosmetic.publish(self.cos_joints)
        except KeyboardInterrupt:
            print("KeyboardInterrupt: Stopping the play dead action.")                
        r.sleep()

if __name__== '__main__':
    rospy.init_node('miro_play_dead', disable_signals=True)
    play_dead = PlayDead()
    rospy.spin()
    rospy.loginfo("Play dead node is running.")