#!/usr/bin/env python
#-*- encoding: utf8 -*-

import rospy

from threading import Thread
import time

from std_msgs.msg import Bool
from mhri_msgs.srv import EmptyResult

from pca9685 import PCA9685

class IdleMotion(Thread):
	def __init__(self):
		Thread.__init__(self)

		self.is_ready = True
		self.pca9685 = PCA9685(0x40, 1, False)
		self.pca9685.setPwmClock(1000)
		rospy.Subscriber('idle_motion/set_status', Bool, self.handle_idle_status)
		self.srv_is_ready = rospy.Service('idle_motion/is_ready', EmptyResult, self.handle_is_ready)
		
		rospy.loginfo('[%s] is ready.'%rospy.get_name())

	def shutdown(self):
		self.pca9685.setLED(0, 0, 0)
		self._run = False
		#rospy.loginfo('[%s] shutdown'%rospy.get_name())
		return

	def run(self):
		self._run = True
		self.count = 0.0
		self.countUp = True
		while self._run:
			#print 'thread run'
			if self.countUp:
				self.count = self.count+0.003
				if self.count > 0.5:
					self.countUp = False
			else:
				self.count = self.count-0.003
				if self.count < 0.1:
					self.countUp = True

			self.pca9685.setLED(0, self.count, self.count)
			time.sleep(0.01)


	def handle_is_ready(self, req):
		return True


	def handle_idle_status(self, msg):
		return

if __name__ == '__main__':
	rospy.init_node('boxy_idle_motion', anonymous=False)
	
	m = IdleMotion()
	m.start()
	m.join()

	rospy.spin()
	
	m.shutdown()
	exit(0)
