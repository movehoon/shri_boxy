#!/usr/bin/env python
#-*- coding: utf-8 -*-

import os
import rospy

import pygame, time
from pygame.locals import *

from threading import Thread
import time

from perception_base.perception_base import PerceptionBase


HEAD_FRONT = 0
BUMPER_LEFT = 1
BUMPER_RIGHT = 2

class KeyboardEvents(PerceptionBase, Thread):
    "Sends callbacks for tactile touch, bumper press and foot contact to ROS"
    def __init__(self):
        super(KeyboardEvents, self).__init__("keyboard_events")

        Thread.__init__(self)

        # ROS initialization:
        rospy.init_node('keyboard_events')

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
        self._run = True
        while self._run:
            #rospy.loginfo('head: ' + str(self.head_touched) + ' prev: ' + str(self.head_touched_prev));

            input_cmd = raw_input()
            # rospy.loginfo('input_cmd: ' + input_cmd)

            if input_cmd == 'h':
                rospy.loginfo('keyboard front head touched')
                write_data = self.conf_data['touch_activity']['data']
                write_data['touched_part'] = HEAD_FRONT
                write_data['state'] = True
                self.save_to_memory(self.conf_data.keys()[0], data=write_data)
                self.raise_event(self.conf_data.keys()[0], 'front_head_touched')

            if input_cmd == 'l':
                rospy.loginfo('keyboard left bumper touched')
                write_data = self.conf_data['touch_activity']['data']
                write_data['touched_part'] = BUMPER_LEFT
                write_data['state'] = True
                self.save_to_memory(self.conf_data.keys()[0], data=write_data)
                self.raise_event(self.conf_data.keys()[0], 'left_bumper_touched')

            if input_cmd == 'r':
                rospy.loginfo('tactile right bumper touched')
                write_data = self.conf_data['touch_activity']['data']
                write_data['touched_part'] = BUMPER_RIGHT
                write_data['state'] = True
                self.save_to_memory(self.conf_data.keys()[0], data=write_data)
                self.raise_event(self.conf_data.keys()[0], 'right_bumper_touched')

            time.sleep(0.1)


    def stop(self):
        self._run = False


if __name__ == '__main__':

    t = KeyboardEvents()
    t.start()

    rospy.spin()

    t.stop()

    exit(0)

