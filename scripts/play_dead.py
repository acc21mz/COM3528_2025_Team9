#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage, Range, Imu
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist, Pose
from datetime import datetime
from sensor_msgs.msg import JointState
import os
from math import radians  

# play dead makes miro close its eyes and point its ears and tail down.

class PlayDead():

    def __init__(self):

        ## Node rate
        self.rate = rospy.get_param('rate',10)
        self.robot_name = os.getenv("MIRO_ROBOT_NAME", "miro")
        self.pub_kinematic = rospy.Publisher(f'/{self.robot_name}/control/kinematic_joints', JointState, queue_size=0)
        self.pub_cosmetic = rospy.Publisher(f'/{self.robot_name}/control/cosmetic_joints', Float32MultiArray, queue_size=0)
        rospy.Subscriber('/miro/control',String, self.control_callback)
        
    def control_callback(self, msg):
        if msg.data.strip().lower() == "play dead":
            self.miro_dead()
    
    def miro_dead(self):

        r = rospy.Rate(self.rate)
        self.kin_joints = JointState()  # Prepare the empty message
        self.cos_joints = Float32MultiArray()  # Prepare the empty message
        try:
            self.kin_joints.name = ["tilt", "lift", "yaw", "pitch"]
            self.kin_joints.position = [0.0, radians(0), 0.0, 0.0]
            #self.cos_joints.name = ["droop", "wag", "eyel", "eyer", "earl", "earr"]
            self.cos_joints.data = [-1.0, 0.0, 1.0, 1.0, -0.5, -0.5]
            self.pub_kinematic.publish(self.kin_joints)
            self.pub_cosmetic.publish(self.cos_joints)
        except KeyboardInterrupt:
            print("KeyboardInterrupt: Stopping the play dead action.")                
        r.sleep()

if __name__== '__main__':
    rospy.init_node('miro_play_dead', disable_signals=True)
    play_dead = PlayDead()
    rospy.spin()
    