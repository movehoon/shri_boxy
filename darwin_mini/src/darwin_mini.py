#!/usr/bin/env python
#-*- coding: utf-8 -*-

import os
from threading import Thread
import time
import yaml
import json
import random
import re

import rospy
import actionlib
import rospkg

import dynamixel_functions as dynamixel                     # Uses Dynamixel SDK library

from mhri_msgs.msg import RenderItemAction, RenderItemResult, RenderItemFeedback
from mhri_msgs.srv import GetInstalledGestures, GetInstalledGesturesResponse

from perception_base.perception_base import PerceptionBase

# Protocol version
PROTOCOL_VERSION            = 2                             # See which protocol version is used in the Dynamixel

# Default setting
DXL_ID                      = 200                             # Dynamixel ID: 200
BAUDRATE                    = 57600
DEVICENAME                  = "/dev/ttyUSB0".encode('utf-8')# Check which port is being used on your controller
                                                            # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"
COMM_SUCCESS                = 0                             # Communication Success result value
COMM_TX_FAIL                = -1001                         # Communication Tx Failed


BUMPER_LEFT = 1
BUMPER_RIGHT = 2


class DarwinBehaviors(PerceptionBase, Thread):

    def __init__(self):

        super(DarwinBehaviors, self).__init__("darwin_mini_task")

        Thread.__init__(self)

        # Initialize PortHandler Structs
        # Set the port path
        # Get methods and members of PortHandlerLinux or PortHandlerWindows
        self.port_num = dynamixel.portHandler(DEVICENAME)

        # Initialize PacketHandler Structs
        dynamixel.packetHandler()

        dxl_comm_result = COMM_TX_FAIL                              # Communication result

        # Open port
        if not dynamixel.openPort(self.port_num):
            rospy.logerr("Failed to open the port!")

        # Set port baudrate
        if not dynamixel.setBaudRate(self.port_num, BAUDRATE):
            rospy.logerr("Failed to change the baudrate!")

        for x in range(3):
            rospy.loginfo('[%s] for loop [%d]' %(rospy.get_name(),x))
            dxl_model_number = dynamixel.pingGetModelNum(self.port_num, PROTOCOL_VERSION, DXL_ID)
            if dynamixel.getLastTxRxResult(self.port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
                dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(self.port_num, PROTOCOL_VERSION))
            elif dynamixel.getLastRxPacketError(self.port_num, PROTOCOL_VERSION) != 0:
                dynamixel.printRxPacketError(PROTOCOL_VERSION, dynamixel.getLastRxPacketError(self.port_num, PROTOCOL_VERSION))
            elif dynamixel.getLastTxRxResult(self.port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
                rospy.loginfo('[%s] Got model number [%d] from ping!' %(rospy.get_name(),dxl_model_number))
                break
            rospy.sleep(0.5)


        self.behavior = None
        self.gesture = ''

        try:
            rospy.get_param('~motion_file')
        except KeyError:
            rospy.logerr('Set parammeter ~motion_file')
            quit(1)

        motion_file = os.path.join(rospkg.RosPack().get_path('darwin_mini'), 'config', 'motions.yaml')
        stream = file(motion_file, 'r')
        self.motion_list = yaml.load(stream)

        self.getInstalledGesturesService = rospy.Service(
            "get_installed_gestures",
            GetInstalledGestures,
            self.getInstalledGestures
        )

        # Prepare and start actionlib server
        self.actionlibServer = actionlib.SimpleActionServer(
            "render_gesture",
            RenderItemAction,
            self.render_gesture,
            False
        )

        self.actionlibServer.register_preempt_callback(self.stop_gesture)
        self.actionlibServer.start()

        rospy.loginfo('[%s] is ready.'%rospy.get_name())
        # rospy.spin()



    def getInstalledGestures(self, request):
        result = json.dumps(self.motion_list)
        return GetInstalledGesturesResponse(result)



    def render_gesture(self, goal):
        rospy.loginfo("Execution of behavior: '{}' requested".format(goal.data))

        result = RenderItemResult()
        feedback = RenderItemFeedback()
        succeed = True

        gesture_type, gesture_data = goal.data.split('=')
        if gesture_type == 'gesture':
            (cmd, item_name) = gesture_data.split(':')
            if cmd == 'tag':
                match = re.search(r'\[(.+?)\]', item_name)
                rendering_gesture = ''
                if match:
                    item_name = item_name.replace(match.group(0), '')
                    emotion = match.group(1)

                    try:
                        rendering_gesture = self.motion_list[item_name][emotion][random.randrange(0, len(self.motion_list[item_name]) - 1)]
                    except (KeyError, TypeError):
                        rendering_gesture = self.motion_list[item_name][random.randint(0, len(self.motion_list[item_name]) - 1)]

                else:
                    try:
                        rendering_gesture = self.motion_list[item_name][random.randint(0, len(self.motion_list[item_name]) - 1)]
                    except KeyError:
                        rendering_gesture = self.motion_list['neutral'][random.randint(0, len(self.motion_list[item_name]) - 1)]

                if self.actionlibServer.is_preempt_requested():
                    self.actionlibServer.set_preempted()
                    rospy.logdebug("Gesture execution preempted before it started")
                    return

                self.gesture = rendering_gesture

                rospy.logdebug("Waiting for behavior execution to complete")

            elif cmd == 'play':
                found_motion = False
                for k, v in self.motion_list.items():
                    if item_name in v:
                        found_motion = True

                if not found_motion:
                    error_msg = "Gesture '{}' not installed".format(item_name)
                    self.actionlibServer.set_aborted(text = error_msg)
                    rospy.logerr(error_msg)
                    return
                else:
                    self.gesture = item_name
                    motion_type = self.gesture.split('/')
                    if (len(motion_type) > 2):
                        if motion_type[1] == 'greeting':
                            rospy.loginfo("Execution of gesture: '{}' requested".format(motion_type[1]))
                            dynamixel.write1ByteTxRx(self.port_num, PROTOCOL_VERSION, DXL_ID, 66, 5)
                            # dynamixel.write1ByteTxRx(self.port_num, PROTOCOL_VERSION, DXL_ID, 66, 6)
                            # dynamixel.write1ByteTxRx(self.port_num, PROTOCOL_VERSION, DXL_ID, 66, 13)
                            # dynamixel.write1ByteTxRx(self.port_num, PROTOCOL_VERSION, DXL_ID, 66, 14)

                        elif motion_type[1] == 'waiting':
                            dynamixel.write1ByteTxRx(self.port_num, PROTOCOL_VERSION, DXL_ID, 66, 3)
                            pass

                        elif motion_type[1] == 'user_defined':

                            if motion_type[2] == 'angry':
                                dynamixel.write1ByteTxRx(self.port_num, PROTOCOL_VERSION, DXL_ID, 66, 7)
                                # dynamixel.write1ByteTxRx(self.port_num, PROTOCOL_VERSION, DXL_ID, 66, 8)
                                
                            elif motion_type[2] == 'bye':
                                dynamixel.write1ByteTxRx(self.port_num, PROTOCOL_VERSION, DXL_ID, 66, 13)

                        succeed = True

        else:
            succeed = False

        rospy.sleep(1)

        if succeed:
            result.result = True
            self.actionlibServer.set_succeeded(result)
            rospy.loginfo('\033[95m%s\033[0m rendering completed...' % rospy.get_name())

    def stop_gesture(self):
        if self.gesture and self.actionlibServer.is_active():
            pass





    def shutdown(self):
        self._run = False
        return

    def run(self):
        self._run = True

        self.left_touched = 0
        self.right_touched = 0

        while self._run:

            touch_l = dynamixel.read1ByteTxRx(self.port_num, PROTOCOL_VERSION, DXL_ID, 378)
            touch_r = dynamixel.read1ByteTxRx(self.port_num, PROTOCOL_VERSION, DXL_ID, 379)
    
            if self.left_touched == 0 and touch_l > 0:
                rospy.loginfo('keyboard left bumper touched')
                write_data = self.conf_data['touch_activity']['data']
                write_data['touched_part'] = BUMPER_LEFT
                write_data['state'] = True
                self.save_to_memory(self.conf_data.keys()[0], data=write_data)
                self.raise_event(self.conf_data.keys()[0], 'left_bumper_touched')

            if self.right_touched == 0 and touch_r > 0:
                rospy.loginfo('tactile right bumper touched')
                write_data = self.conf_data['touch_activity']['data']
                write_data['touched_part'] = BUMPER_RIGHT
                write_data['state'] = True
                self.save_to_memory(self.conf_data.keys()[0], data=write_data)
                self.raise_event(self.conf_data.keys()[0], 'right_bumper_touched')

            self.left_touched = touch_l
            self.right_touched = touch_r

            time.sleep(0.3)


    def stop(self):
        self._run = False


if __name__ == '__main__':
    node = DarwinBehaviors()
    node.start()

    rospy.spin()

    node.stop()

    exit(0)
