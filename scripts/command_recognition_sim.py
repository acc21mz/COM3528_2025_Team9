#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

# Dummy class to simulate platform_control since we don't have MiRo
class DummyPlatformControl:
    def __init__(self):
        self.eyelid_closure = 0.0
        self.lights_raw = []
        self.tail = 0.0
        self.ear_rotate = []
        self.body_config = []
        self.body_config_speed = []
        self.body_vel = Twist()

class CommandRecognition:

    def __init__(self):
        self.rate = rospy.get_param('rate', 200)
        self.robot_name = rospy.get_param('/robot_name', 'sim01')
        self.topic_root = "/miro/" + self.robot_name
        print("topic_root", self.topic_root)

        self.activate = False
        self.command = ""

        # Simulated control messages
        self.q_sleep = DummyPlatformControl()
        self.q_sad = DummyPlatformControl()
        self.q_play = DummyPlatformControl()
        self.q_gbb = DummyPlatformControl()
        self.q_good = DummyPlatformControl()
        self.q_kill = DummyPlatformControl()
        
        # subscriber to receive commands from speech recognition
        self.sub_speech_recognition = rospy.Subscriber(self.topic_root + "/speech_to_text", String, self.callback_receive_command, queue_size=1)

        # Publisher to simulate robot control topic
        self.pub_platform_control = rospy.Publisher(self.topic_root + "/platform/control", String, queue_size=1)

        # Publisher for velocity commands (to make robot spin)
        self.pub_velocity = rospy.Publisher(self.topic_root + "/platform/velocity", Twist, queue_size=1)

        # Publisher for unrecognised commands
        self.unrecognised_pub = rospy.Publisher('/unrecognised_command_trigger', String, queue_size=1)

    def callback_receive_command(self, text):
        print(text)
        self.command = text.data.strip().lower()
        print(f"\U0001F5E3️ Heard: {self.command}")
        
    def spin_once(self):
        twist = Twist()
        twist.angular.z = 1.0  # Spin in place
        self.pub_velocity.publish(twist)
        rospy.sleep(0.5)  # Short spin burst
        twist.angular.z = 0.0
        self.pub_velocity.publish(twist)
        
    def get_command(self):
        return self.command

    def switching_commands(self):
        r = rospy.Rate(self.rate)
        count_bad = 0
        count_miro = 0
        count_sleep = 0

        while not rospy.is_shutdown():
            
            response = None
            #self.command = (self.sub_speech_recognition.callback)
            if self.command != "":
                if self.command in ["miro", "myro", "myra"]:
                    count_miro += 1
                    count_sleep = 0
                    if count_miro >= 1:
                        response = "[MIRO ACTIVATED]"
                    self.activate = True

                elif self.activate and self.command == "sleep":
                    count_miro = 0
                    count_bad = 0
                    self.activate = False
                    count_sleep = 1
                    response = "[SLEEP MODE ACTIVATED]"

                elif self.activate and self.command == "bad":
                    count_bad += 1
                    if count_bad < 2000:
                        response = f"[BAD COMMAND TRIGGERED - count {count_bad}]"
                    else:
                        response = "[BAD MAX REACHED - Lights RED]"

                elif self.activate and self.command == "play":
                    count_bad = 0
                    response = "[PLAY MODE TRIGGERED]"

                elif self.activate and self.command == "let's go out":
                    count_bad = 0
                    response = "[LET'S GO OUT TRIGGERED]"

                elif self.activate and self.command == "good":
                    count_bad = 0
                    response = "[GOOD COMMAND TRIGGERED]"

                elif self.activate and self.command == "kill":
                    count_bad = 0
                    response = "[KILL COMMAND TRIGGERED]"

                elif count_sleep == 1 and not self.activate and self.command != "sleep":
                    response = "[ROBOT SLEEPING - Ignoring Command]"

                if response:
                    rospy.loginfo(response)
                    self.pub_platform_control.publish(response)
                    self.spin_once()
                    self.command = ""
                else:
                    # Only runs if no recognised command matched
                    valid_commands = [
                        "miro", "myro", "myra", "sleep", "bad", "play", "let's go out", "good", "kill", "fetch"
                    ]
                    if self.activate and self.command and self.command not in valid_commands:
                        rospy.loginfo(f"Unrecognised command: {self.command}")
                        print("Publishing to /unrecognised_command_trigger:", self.command)
                        self.unrecognised_pub.publish(String(data=self.command))
                        self.command = ""
                r.sleep()

    def trigger_callback(self, msg):
        self.unrecognised_response()

if __name__ == '__main__':
    rospy.init_node('command_recognition')
    cr = CommandRecognition()
    cr.switching_commands()