#!/usr/bin/env python3

################################################################

##DO NOT USE, NOT OUR CODE##

import rospy
from std_msgs.msg import String,Bool,Int32
from sensor_msgs.msg import Image,CompressedImage,Range,Imu
from geometry_msgs.msg import Twist,Pose

#only when testing on real miro uncomment these
import miro_msgs
from miro2_msgs.msg import platform_control
from miro_constants import miro
import opencv_apps
from opencv_apps.msg import CircleArrayStamped

from datetime import datetime

class CommandRecognition():

    ## Constructor   
    def __init__(self):

        ## Node rate
        self.rate = rospy.get_param('rate',200)

        ## Allow to switch from real robot to simulation from launch file
        #self.robot_name = rospy.get_param ( '/robot_name', 'dia-miro12')
        self.robot_name = rospy.get_param ( '/robot_name', 'sim01')
        self.topic_root = "/miro/" + self.robot_name
        print("topic_root", self.topic_root)

        self.activate = False
        self.command = String()

        # platform control Objects for all actions 
        self.q_play_dead = platform_control()
        self.q_stop = platform_control()
        self.q_fetch = platform_control()
        self.q_follow_me = platform_control()
        self.q_speak = platform_control()
        
        # subscriber to speech-to-text
        self.sub_speech_to_text = rospy.Subscriber(self.topic_root + '/speech_to_text', String, self.callback_receive_command,queue_size=1)
        
        self.sub_play_dead_action = rospy.Subscriber('/miro_play_dead', platform_control, self.callback_play_dead_action,queue_size=1)
        self.sub_stop_action = rospy.Subscriber('/miro_stop', platform_control, self.callback_stop_action,queue_size=1) 
        self.sub_fetch_action = rospy.Subscriber('/miro_fetch', platform_control, self.callback_fetch_action,queue_size=1) 
        self.sub_follow_me_action = rospy.Subscriber('/miro_follow_me', platform_control, self.callback_follow_me_action,queue_size=1) 
        self.sub_speak_action = rospy.Subscriber('/miro_speak', platform_control, self.callback_speak_action,queue_size=1) 

        ## Publisher to the topic /platform/control a message of type platform_control which execute Miro actions 
        self.pub_platform_control = rospy.Publisher(self.topic_root + "/platform/control", platform_control, queue_size=0)
    
    
    ## Callback function that receive and save the user's voice command as text
    def callback_receive_command(self, text):
        self.command = text.data
        print(f"🗣️ Heard: {text.data}")
    
    def callback_play_dead_action(self, play_dead):
        self.q_play_dead = play_dead

    def callback_stop_action(self, stop):
        self.q_stop = stop
    
    def callback_fetch_action(self, fetch):
        self.q_fetch = fetch
        self.q_fetch.body_config = [0.0,0.0,0.0,0.0]
        self.q_fetch.body_config_speed = [0.0,-1.0,-1.0,-1.0]
    
    def callback_follow_me_action(self, follow_me):
        self.q_good = follow_me
    
    def callback_speak_action(self, speak):

        self.q_speak = speak
    
    
    def switching_commands(self):

        q = platform_control()
        q.eyelid_closure = 1.0

        r = rospy.Rate(self.rate)
        count_miro = 0
        
        while not rospy.is_shutdown():

            if self.command == "Miro" or self.command == " Miro" or self.command == "miro" or self.command == " miro":
                count_miro = 0
                count_miro = count_miro +1
                rospy.loginfo(count_miro)

                if count_miro == 1:
                    q.eyelid_closure = 0.0
                    q.lights_raw = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                    q.tail = -1.0
                    q.ear_rotate = [0.0,0.0]
                    q.body_config = [0.0,0.0,0.0,0.0]
                    q.body_config_speed = [0.0,-1.0,-1.0,-1.0]
                    self.pub_platform_control.publish(q)
                    
                self.activate = True

            # play_dead
            if self.activate and self.command == "play dead" or self.command == "play dad":
                count_miro = 0
                q = self.q_play_dead
                self.pub_platform_control.publish(q)
                self.activate = False
                print("Play dead")
            
            # stop
            elif self.activate and (self.command == "Stop" or self.command == "stop"):
                q = self.q_stop
                q.body_vel.linear.x = 0.0
                q.body_vel.angular.z = 0.0
                self.pub_platform_control.publish(q)
                self.activate = False
                print("Stop")
            
            # fetch
            elif self.activate and (self.command == "Fetch" or self.command == "fetch"):
                q = self.q_fetch
                self.pub_platform_control.publish(q)

            # follow me
            elif self.activate and (self.command == "follow me" or self.command == "Follow Me"):
                count_bad = 0
                q.body_config = [0.0,0.0,0.0,0.0]
                q.body_config_speed = [0.0,-1.0,-1.0,-1.0]
                q = self.q_gbb
                self.pub_platform_control.publish(q)  
                print("follow me")

            # speak
            elif self.activate and (self.command == "Speak" or self.command == "speak"):
                q = self.q_speak
                self.pub_platform_control.publish(q)  
                print("speak")

            r.sleep()

if __name__== '__main__':

    rospy.init_node('command_recognition')
    sb = CommandRecognition()
    sb.switching_commands()