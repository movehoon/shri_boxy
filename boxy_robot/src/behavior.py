#!/usr/bin/env python
#-*- coding: utf-8 -*-

import os
import threading
import yaml

import rospy
import actionlib
import rospkg
import json

from mhri_msgs.msg import RenderItemAction, RenderItemResult, RenderItemFeedback
from mhri_msgs.srv import GetInstalledGestures, GetInstalledGesturesResponse

from pca9685 import PCA9685

class BoxyBehaviors():
    #This should be treated as a constant
    NODE_NAME = "boxy_behavior"

    def __init__( self ):

        #We need this variable to be able to call stop behavior when preempted
        self.behavior = None

        #Load Behavior Tag
        self.motion_list = {
                'forward': ['forward'],
                'stop': ['stop'],
                'right_turn': ['right_turn'],
                'left_turn': ['left_turn'],
            }

        # Register ROS services
        self.GetInstalledGesturesService = rospy.Service(
            "get_installed_gestures",
            GetInstalledGestures,
            self.handle_get_installed_gestures
        )

        #Prepare and start actionlib server
        self.server = actionlib.SimpleActionServer(
            'render_gesture', RenderItemAction, self.execute_callback, False)
        # self.server.register_preempt_callback(self.preempted_callback)
        self.server.start()


        self.pca9685 = PCA9685(0x40, 0, False)
        self.pca9685.setPwmClock(1000)
        self.pca9685.enMotor(True)

        rospy.loginfo('[%s] is ready.'%rospy.get_name())

    def __del__(self):
        self.pca9685.enMotor(False)
	return

    def handle_get_installed_gestures(self, req):
        result = json.dumps(self.motion_list)
        return GetInstalledGesturesResponse( result )


    def execute_callback(self, goal):
        #Note this function is executed from a different thread
        rospy.loginfo('\033[95m%s\033[0m rendering requested...' % rospy.get_name())

        result = RenderItemResult()
        feedback = RenderItemFeedback()

        (gesture_category, gesture_item) = goal.data.split('/')

        if gesture_category == 'mobility':
            (cmd, item_name) = gesture_item.split(':')
            if cmd == 'move':
                rospy.loginfo('\033[94m[%s]\033[0m rendering gesture cmd [%s], name [%s]...'%(rospy.get_name(), cmd, item_name))
                if item_name == 'forward':
                    print ('move forward')
                    self.pca9685.motorSpeed(0,  0.5)
                    self.pca9685.motorSpeed(1, -0.5)
                elif item_name == 'stop':
                    print ('move stop')
                    self.pca9685.motorSpeed(0, 0)
                    self.pca9685.motorSpeed(1, 0)

#        if request.gesture == 'forward':
#            self.pca9685.motorSpeed(0,  0.5)
#            self.pca9685.motorSpeed(1, -0.5)
#        if request.gesture == 'stop':
#            self.pca9685.motorSpeed(0, 0)
#            self.pca9685.motorSpeed(1, 0)


        # Send Feedback first.
        feedback.is_rendering = True
        self.server.publish_feedback(feedback)

        result.result = True
        self.server.set_succeeded(result)



    def stopGesture( self ):
#        with self.lock:
        if self.gesture and self.actionlibServer.is_active() :
            rospy.logdebug("StopGesture")
            # self.behaviorProxy.stopBehavior( self.gesture )
            self.pca9685.motorSpeed(0, 0)
            self.pca9685.motorSpeed(1, 0)


if __name__ == '__main__':
    rospy.init_node('boxy_tasks', anonymous=False)
    node = BoxyBehaviors()
    rospy.loginfo( node.NODE_NAME + " running..." )
    rospy.spin()
    rospy.loginfo( node.NODE_NAME + " stopped." )
    exit(0)
