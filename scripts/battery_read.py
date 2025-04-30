#!/usr/bin/env python

import rospy
rospy.init_node("client_test", anonymous=True)

import os
import sys
import time
import numpy as np

import miro2 as miro

"""
CheckBattery class:
Checks the battery and alerts if the battery % falls below 10 by playing an audio
% is calculated by relative voltage; 0% = 3.0V, 100% = 
"""
class CheckBattery:

	def callback_battery(self, msg):

		# Print battery percentage
		print("Current battery percentage: " + msg.voltage) 
		

	def loop(self):

		# loop for a while
		t_now = 0
		while t_now < 3.0 and not rospy.core.is_shutdown():
			time.sleep(0.1)
			t_now += 0.1
		# add code here to deal with the % < 10

	def __init__(self, args):
	
		# state
		self.vbat = 0

		# robot name
		topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")

		# subscribe
		topic = topic_base_name + "/sensors/battery"
		print ("subscribe", topic)
		self.sub_battery = rospy.Subscriber(topic, miro.msg.BatteryState,
					self.callback_battery, queue_size=1, tcp_nodelay=True)

		# wait for connect
		time.sleep(1)

if __name__ == "__main__":

	# Check the battery and loop (should be done throughout the program)
	main = CheckBattery(sys.argv[1:])
	main.loop()