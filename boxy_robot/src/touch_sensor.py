#!/usr/bin/env python
#-*- coding: utf-8 -*-

import os
import rospy

from threading import Thread
import time

from perception_base.perception_base import PerceptionBase


HEAD_FRONT = 0
BUMPER_LEFT = 1
BUMPER_RIGHT = 2

PIN_HEAD = '6'
PIN_LEFT_ARM = '10'
PIN_RIGHT_ARM = '11'

class BoxyTouch(PerceptionBase, Thread):
    "Sends callbacks for tactile touch, bumper press and foot contact to ROS"
    def __init__(self):
        super(BoxyTouch, self).__init__("boxy_touch_sensor")

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

            if self.front_head_touched and not self.front_head_touched_prev:
                rospy.loginfo('tactile front head touched');
	        write_data = self.conf_data['touch_activity']['data']
                write_data['touched_part'] = HEAD_FRONT
                write_data['state'] = not self.front_head_touched
                self.save_to_memory(self.conf_data.keys()[0], data=write_data)
                self.raise_event(self.conf_data.keys()[0], 'front_head_touched')

            if self.left_bumper_touched and not self.left_bumper_touched_prev:
                rospy.loginfo('tactile left bumper touched');
                write_data = self.conf_data['touch_activity']['data']
                write_data['touched_part'] = BUMPER_LEFT
                write_data['state'] = not self.left_bumper_touched
                self.save_to_memory(self.conf_data.keys()[0], data=write_data)
                self.raise_event(self.conf_data.keys()[0], 'left_bumper_touched')

            if self.right_bumper_touched and not self.right_bumper_touched_prev:
                rospy.loginfo('tactile right bumper touched');
                write_data = self.conf_data['touch_activity']['data']
                write_data['touched_part'] = BUMPER_RIGHT
                write_data['state'] = not self.right_bumper_touched
                self.save_to_memory(self.conf_data.keys()[0], data=write_data)
                self.raise_event(self.conf_data.keys()[0], 'right_bumper_touched')

            self.backupTouchStatus()
            time.sleep(0.1)


    def stop(self):
        self._run = False


    def refreshTouchStatus(self):
        self.front_head_touched = int(os.popen('gpio read ' + PIN_HEAD).read())
        self.left_bumper_touched = int(os.popen('gpio read ' + PIN_LEFT_ARM).read())
        self.right_bumper_touched = int(os.popen('gpio read ' + PIN_RIGHT_ARM).read())


    def backupTouchStatus(self):
        self.front_head_touched_prev = self.front_head_touched
        self.left_bumper_touched_prev = self.left_bumper_touched
        self.right_bumper_touched_prev = self.right_bumper_touched
 

if __name__ == '__main__':

    t = BoxyTouch()
    t.start()

    rospy.spin()

    t.stop()

    exit(0)

