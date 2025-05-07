#!/usr/bin/env python3

################################################################

##DO NOT USE, NOT OUR CODE##

import rospy
from std_msgs.msg import String,Bool,Int32
from sensor_msgs.msg import Image,CompressedImage,Range,Imu
from geometry_msgs.msg import Twist,Pose

#only when testing on real miro uncomment these
import miro_msgs
from miro_msgs.msg import platform_config,platform_sensors,platform_state,platform_mics,platform_control,core_state,core_control,core_config,bridge_config,bridge_stream
from miro_constants import miro
import opencv_apps
from opencv_apps.msg import CircleArrayStamped

# import math
# import numpy
# import time
# import sys
from datetime import datetime

## \file command_recognition.py 
## \brief The node command_recognition.py recognize vocal command converted as String and publish the associated robot's action
## @n The vocal command "Miro" bring the robot in a default configuration and enables the processing of further commands.
## @n Different commands are managed. Each command publishes a message of type platform_control with the corresponding action.
## @n The action are handled independently by separated nodes. 
## @n The current node subscribes to them and, when the associated command is received, publishes on MiRo the msg content.

##\brief The class CommandRecognition handles the different incoming commands and publish on the robot the corresponding action
class CommandRecognition():

    ## Constructor   
    def __init__(self):

        ## Node rate
        self.rate = rospy.get_param('rate',200)

        #topic root
        ## Allow to switch from real robot to simulation from launch file
        self.robot_name = rospy.get_param ( '/robot_name', 'dia-miro12')
        self.topic_root = "/miro/" + self.robot_name
        print("topic_root", self.topic_root)

        ## Initialization of the enabling command
        self.activate = False
        ## Initialization of the string to evaluate
        self.command = String()
        
        #------------------ ADD OBJECTS OF NEW COMMANDS ----------------------#

        ## Platform control Object that represents the sleep action ("Sleep")
        self.q_play_dead = platform_control()
        ## Platform control Object that represents the miro action when it is scolded ("Bad")
        self.q_sad = platform_control()
        ## Platform control Object that represents the miro action when it follows the Ball ("Play")
        self.q_play = platform_control()
        ## platform_control object that rapresents the Gesture Based Behavior ("Let's go out")
        self.q_gbb = platform_control()
        ## Platform control Object that represents the miro action when it is praised ("Good")
        self.q_good = platform_control()
        ## Platform control Object that represents the miro action when it is praised ("Kill")
        self.q_kill = platform_control()

        ## Subscriber to the topic /speech_to_text a message of type String that cointains the vocal command converted in text
        self.sub_speech_to_text = rospy.Subscriber('/speech_to_text', String, self.callback_receive_command,queue_size=1)

        #------------------ ADD SUBSCRIBERS TO NEW COMMANDS -------------------#
        ## Subscriber to the topic /miro_sleep a message of type platform_control that rapresents the action corresponting to the command "play_dead"
        self.sub_play_dead_action = rospy.Subscriber('/miro_play_dead', platform_control, self.callback_play_dead_action,queue_size=1)
        ## Subscriber to the topic /miro_sad a message of type platform_control that rapresents the action corresponting to the command "Bad"
        self.sub_sad_action = rospy.Subscriber('/miro_sad', platform_control, self.callback_sad_action,queue_size=1) 
        ## Subscriber to the topic /miro_follow a message of type platform_control that rapresents the action corresponting to the command "Play"
        self.sub_play_action = rospy.Subscriber('/miro_play', platform_control, self.callback_play_action,queue_size=1) 
        ## Subscriber to the topic /miro_dance a message of type platform_control that rapresents the action corresponting to the command "Let's go out"
        self.sub_gbb = rospy.Subscriber('/gbb', platform_control, self.callback_gbb,queue_size=1) 
        ## Subscriber to the topic /miro_good a message of type platform_control that rapresents the action corresponting to the command "Good"
        self.sub_good_action = rospy.Subscriber('/miro_good', platform_control, self.callback_good_action,queue_size=1) 
        ## Subscriber to the topic /miro_good a message of type platform_control that rapresents the action corresponting to the command "Kill"
        self.sub_kill_action = rospy.Subscriber('/miro_kill', platform_control, self.callback_kill_action,queue_size=1) 

        ## Publisher to the topic /platform/control a message of type platform_control which execute Miro actions 
        self.pub_platform_control = rospy.Publisher(topic_root + "/platform/control", platform_control, queue_size=0)
    
    
    ## Callback function that receive and save the user's voice command as text
    def callback_receive_command(self, text):

        self.command = text.data
        print(f"🗣️ Heard: {text.data}")
    #------------------ ADD CALLBACK FOR NEW COMMANDS -------------------#    
    ## Callback that receives the action to be executed when the vocal command is "play_dead"
    def callback_play_dead_action(self, play_dead):

        self.q_play_dead = play_dead

    ## Callback that receives the action to be executed when the vocal command is "Bad"
    def callback_sad_action(self, sad):

        self.q_sad = sad
    
    ## Callback that receives the action to be executed when the vocal command is "Play"
    def callback_play_action(self, play):

        self.q_play = play
        self.q_play.body_config = [0.0,0.0,0.0,0.0]
        self.q_play.body_config_speed = [0.0,-1.0,-1.0,-1.0]
    
    ## Callback that receives the action to be executed when the vocal command is "Let's go"
    def callback_gbb(self,gbb):

        self.q_gbb = gbb
    
    ## Callback that receives the action to be executed when the vocal command is "Good"
    def callback_good_action(self, good):

        self.q_good = good
    
    ## Callback that receives the action to be executed when the vocal command is "Kill"
    def callback_kill_action(self, kill):

        self.q_kill = kill
    
    ## Function that check the incoming commands and publish the corresponding action
    ## @n The command "Miro" brings the robot in a default configuration the first time is used. The variable activate is set to True and enables the evauation of the other commands.
    ## @n The command "Bad" is executed only if self.active is True and publish the action managed by the node bad.py
    ## @n The command "Good" is executed only if self.active is True and publish the action managed by the node good.py
    ## @n The command "Play" is executed only if self.active is True and publish the action managed by the node play.py
    ## @n The command "Let's go out" is executed only if self.active is True and publish the action managed by the node gbb_miro.py
    ## @n The command "play_dead" is executed only if self.active is True and publish the action managed by the node play_dead.py. 
    ## @n The variable activate is set to False and Miro remains in play_dead mode until a new command "Miro" is received.
    def switching_commands(self):

        q = platform_control()
        q.eyelid_closure = 1.0

        r = rospy.Rate(self.rate)
        count_bad = 0
        count_miro = 0
        count_play_dead = 0
        
        while not rospy.is_shutdown():

            #ACTIVATION COMMAND

            if self.command == "Miro" or self.command == " Miro" or self.command == "miro" or self.command == " miro":
                count_miro = 0
                count_miro = count_miro +1
                rospy.loginfo(count_miro)
                count_play_dead = 0

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
            if self.activate and self.command == "play dead" or self.command == " Play dead" or self.command == "play Dead" or self.command == " PLAY DEAD":
                count_miro = 0
                count_bad = 0
                q = self.q_play_dead
                self.pub_platform_control.publish(q)
                self.activate = False
                count_play_dead = 1
                print("Play dead")
            
            # BAD
            elif self.activate and (self.command == "Bad" or self.command == " Bad" or  self.command == "bad" or self.command == " bad"):
                count_bad = count_bad + 1
                rospy.loginfo(count_bad)
                if count_bad < 2000:
                    q = self.q_sad
                    self.pub_platform_control.publish(q)
                else:
                    q.body_vel.linear.x = 0.0
                    q.body_vel.angular.z = 0.0
                    q.lights_raw = [255,0,0,255,0,0,255,0,0,255,0,0,255,0,0,255,0,0]
                    self.pub_platform_control.publish(q)
                print("Bad")
            
            # PLAY
            elif self.activate and (self.command == "Play" or self.command == " Play" or self.command == "play" or self.command == " play"):
                count_bad = 0
                q = self.q_play
                self.pub_platform_control.publish(q)

            # LET'S GO OUT
            elif self.activate and (self.command == "Let's go out" or self.command == " Let's go out" or self.command == "let's go out" or self.command == " let's go out"):
                count_bad = 0
                q.body_config = [0.0,0.0,0.0,0.0]
                q.body_config_speed = [0.0,-1.0,-1.0,-1.0]
                q = self.q_gbb
                self.pub_platform_control.publish(q)  
                print("Let's go out")

            # GOOD
            elif self.activate and (self.command == "Good" or self.command == " Good" or self.command == "good" or self.command == " good"):
                count_bad = 0
                q = self.q_good
                self.pub_platform_control.publish(q)  
                print("Good")

            # KILL
            elif self.activate and (self.command == "Kill" or self.command == " Kill" or self.command == "kill" or self.command == " kill"):
                count_bad = 0
                q = self.q_kill
                self.pub_platform_control.publish(q)  
                print("Kill")

            # HANDLING OF DIFFERENT COMMANDS
            elif count_sleep == 1 and (not self.activate and not self.command == "Sleep"):
                rospy.loginfo(count_sleep)
                q.eyelid_closure = 1
                self.pub_platform_control.publish(q)

            r.sleep()

if __name__== '__main__':

    rospy.init_node('command_recognition')
    sb = CommandRecognition()
    sb.switching_commands()