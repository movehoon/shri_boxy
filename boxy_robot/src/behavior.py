#!/usr/bin/env python
#-*- coding: utf-8 -*-

import os
import threading
import yaml

import rospy
import actionlib
import rospkg
import json

from mhri_msgs.msg import GestureActionAction, GestureActionResult, GestureActionFeedback
from mhri_msgs.srv import GetInstalledGestures, GetInstalledGesturesResponse

from pca9685 import PCA9685

class BoxyBehaviors():
    #This should be treated as a constant
    NODE_NAME = "boxy_behavior"

    def __init__( self ):

        #We need this variable to be able to call stop behavior when preempted
        self.behavior = None

        #Load Behavior Tag
        motion_file = os.path.join(rospkg.RosPack().get_path('boxy_tasks'), 'config', 'motion_tag.yaml')
        stream = file(motion_file, 'r')
        self.motion_list = yaml.load(stream)        

        # Register ROS services
        self.getInstalledGesturesService = rospy.Service(
            "get_installed_gestures",
            GetInstalledGestures,
            self.getInstalledGestures
            )

        #Prepare and start actionlib server
        self.actionlibServer = actionlib.SimpleActionServer(
            "run_gesture",
            GestureActionAction,
            self.runGesture,
            False
            )

        self.pca9685 = PCA9685(0x40, 1, False)
        self.pca9685.setPwmClock(1000)
        self.pca9685.enMotor(True)

        self.actionlibServer.register_preempt_callback( self.stopGesture )
        self.actionlibServer.start()

        rospy.loginfo('[%s] is ready.'%rospy.get_name())

    def __del__(self):
        self.pca9685.enMotor(False)
	return

    def getInstalledGestures( self, request ):
        #result = self.behaviorProxy.getInstalledBehaviors()
        result = json.dumps(self.motion_list)
        return GetInstalledGesturesResponse( result )


    def runGesture( self, request ):
        #Note this function is executed from a different thread
        rospy.loginfo(
            "Execution of behavior: '{}' requested".format(request.gesture))

        result = GestureActionResult()
        feedback = GestureActionFeedback()

        if request.gesture == 'forward':
            self.pca9685.motorSpeed(0,  0.5)
            self.pca9685.motorSpeed(1, -0.5)
        elif request.gesture == 'stop':
            self.pca9685.motorSpeed(0, 0)
            self.pca9685.motorSpeed(1, 0)

        # #Check requested gesture is installed
        # if not request.gesture in self.behaviorProxy.getInstalledBehaviors():
        #     error_msg = "Gesture '{}' not installed".format(request.gesture)
        #     self.actionlibServer.set_aborted(text = error_msg)
        #     rospy.logerr(error_msg)
        #     return

#        with self.lock:
        # Check first if we're already preempted, and return if so
        if self.actionlibServer.is_preempt_requested():
            self.actionlibServer.set_preempted()
            rospy.loginfo("Gesture execution preempted before it started")
            return

        # Save name of behavior to be run
        self.gesture = request.gesture
        # Execute behavior (on another thread so we can release lock)


        # Wait for task to complete (or be preempted)
        rospy.loginfo("Waiting for behavior execution to complete")
        
        #self.behaviorProxy.wait( taskID, 0 )
#        while dynamixel.read1ByteTxRx(self.port_num, PROTOCOL_VERSION, DXL_ID, ADDR_MOTION_PLAY_STATUS) > 0:
        feedback.is_playing = True
        self.actionlibServer.publish_feedback(feedback)
        rospy.sleep(0.2)

        #Evaluate results
#        with self.lock:
        self.gesture = None
        # If preempted, report so
        if self.actionlibServer.is_preempt_requested() :
            self.actionlibServer.set_preempted()
            rospy.logdebug("Behavior execution preempted")
        # Otherwise, set as succeeded
        else:
            result.result = True
            self.actionlibServer.set_succeeded(result)
            rospy.logdebug("Behavior execution succeeded")


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
