#!/usr/bin/env python
#-*- coding: utf-8 -*-

import os
import rospy

from threading import Thread
import time

from mhri_social_msgs.msg import TouchActivity
from std_msgs.msg import Bool


PIN_HEAD = '6'
PIN_LEFT_ARM = '10'
PIN_RIGHT_ARM = '11'

class BoxyTouch(Thread):
    "Sends callbacks for tactile touch, bumper press and foot contact to ROS"
    def __init__(self):
        Thread.__init__(self)

        os.system('gpio mode ' + PIN_HEAD + ' in')
        os.system('gpio mode ' + PIN_HEAD + ' up')
        os.system('gpio mode ' + PIN_LEFT_ARM + ' in')
        os.system('gpio mode ' + PIN_LEFT_ARM + ' up')
        os.system('gpio mode ' + PIN_RIGHT_ARM + ' in')
        os.system('gpio mode ' + PIN_RIGHT_ARM + ' up')
        self.refreshTouchStatus()
        self.backupTouchStatus()

        # ROS initialization:
        rospy.init_node('boxy_touch_sensor')

        # init. messages:
        self.touch_event = TouchActivity()        
        self.toucheventPub = rospy.Publisher("touch_activity", TouchActivity, queue_size=10)

        rospy.loginfo('[%s] Initialzed'%rospy.get_name())


    def shutdown(self):
        # Shutting down broker seems to be not necessary any more
        # try:
        #     self.broker.shutdown()
        # except RuntimeError,e:
        #     rospy.logwarn("Could not shut down Python Broker: %s", e)
        self._run = False
        return


    def run(self):
        self._run = True;
        while self._run:
            self.refreshTouchStatus()
            #rospy.loginfo('head: ' + str(self.head_touched) + ' prev: ' + str(self.head_touched_prev));

            if self.head_touched != self.head_touched_prev:
                rospy.loginfo('tactile head touched');
                self.touch_event.touched_part = TouchActivity.HEAD
                self.touch_event.state = not self.head_touched
                self.toucheventPub.publish(self.touch_event)

            if self.left_arm_touched != self.left_arm_touched_prev:
                rospy.loginfo('tactile left hand touched');
                self.touch_event.touched_part = TouchActivity.L_HAND
                self.touch_event.state = not self.left_arm_touched
                self.toucheventPub.publish(self.touch_event)

            if self.right_arm_touched != self.right_arm_touched_prev:
                rospy.loginfo('tactile right hand touched');
                self.touch_event.touched_part = TouchActivity.R_HAND
                self.touch_event.state = not self.right_arm_touched
                self.toucheventPub.publish(self.touch_event)

            self.backupTouchStatus()
            time.sleep(0.1)


    def stop(self):
        self._run = False


    def refreshTouchStatus(self):
        self.head_touched = int(os.popen('gpio read ' + PIN_HEAD).read())
        self.left_arm_touched = int(os.popen('gpio read ' + PIN_LEFT_ARM).read())
        self.right_arm_touched = int(os.popen('gpio read ' + PIN_RIGHT_ARM).read())


    def backupTouchStatus(self):
        self.head_touched_prev = self.head_touched
        self.left_arm_touched_prev = self.left_arm_touched
        self.right_arm_touched_prev = self.right_arm_touched
 

if __name__ == '__main__':

    t = BoxyTouch()
    t.start()

    rospy.spin()

    t.stop()

    exit(0)

