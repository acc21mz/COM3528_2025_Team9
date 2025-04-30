#!/usr/bin/env python

import rospy
rospy.init_node("client_test", anonymous=True)

################################################################

import os
import sys
import time
import numpy as np

import miro2 as miro

################################################################

class CheckBattery:

	def callback_battery(self, msg):

		# report battery voltage	
		print(msg.voltage)

	def loop(self):

		# loop for a while
		t_now = 0
		while t_now < 3.0 and not rospy.core.is_shutdown():
			time.sleep(0.1)
			t_now += 0.1

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
		print ("wait for connect...")
		time.sleep(1)

if __name__ == "__main__":

	# normal singular invocation
	main = CheckBattery(sys.argv[1:])
	main.loop()