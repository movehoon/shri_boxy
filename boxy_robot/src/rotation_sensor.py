#!/usr/bin/python

import os
import rospy

from threading import Thread
import time

from perception_core.perception_base import PerceptionBase

from mhri_social_msgs.msg import RotationActivity
from std_msgs.msg import Bool

import smbus
import math

# Power management registers
power_mgmt_1 = 0x6b
power_mgmt_2 = 0x6c

class BoxyRotation(PerceptionBase, Thread):
    "Sends callbacks for rotation changed event"
    def __init__(self):
        super(BoxyRotation, self).__init__("boxy_rotation_sensor")

        Thread.__init__(self)

        # MPU6050 initialization
        self.bus = smbus.SMBus(0) # or bus = smbus.SMBus(1) for Revision 2 boards
        self.address = 0x68       # This is the address value read via the i2cdetect command

        # Now wake the 6050 up as it starts in sleep mode
        self.bus.write_byte_data(self.address, power_mgmt_1, 0)

        self.rotation = RotationActivity.TOP_UP

        # ROS initialization:
        rospy.init_node('boxy_rotation_sensor')

        rospy.loginfo('[%s] Initialzed'%rospy.get_name())


    def shutdown(self):
        self._run = False
        return


    def run(self):
        self._run = True
        while self._run:

            accel_xout = self.read_word_2c(0x3b)
            accel_yout = self.read_word_2c(0x3d)
            accel_zout = self.read_word_2c(0x3f)

            accel_xout_scaled = accel_xout / 16384.0
            accel_yout_scaled = accel_yout / 16384.0
            accel_zout_scaled = accel_zout / 16384.0

            rot_x = self.get_x_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled)
            rot_y = self.get_y_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled)
            rot_z = self.get_z_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled)

            #print ('%0.2f, %0.2f, %0.2f' %(rot_x, rot_y, rot_z))

            rx0 = int(rot_x / 10)
            ry0 = int(rot_y / 10)
            rz0 = int(rot_z / 10)

            rx1 = int(rot_x / 80)
            ry1 = int(rot_y / 80)
            rz1 = int(rot_z / 80)

            write_data = self.conf_data['rotation_activity']['data']
            write_data['rotation_x'] = rot_x
            write_data['rotation_y'] = rot_y
            write_data['rotation_z'] = rot_z

            if rx0==0 and ry0==0 and rz0==0 and self.rotation!=RotationActivity.TOP_UP:
                self.save_to_memory(self.conf_data.keys()[0], data=write_data)
                self.raise_event(self.conf_data.keys()[0], 'top_up')

            elif rx0==0 and ry1==1 and rz1==1 and self.rotation!=RotationActivity.FRONT_UP:
                self.save_to_memory(self.conf_data.keys()[0], data=write_data)
                self.raise_event(self.conf_data.keys()[0], 'front_up')

            elif rx0==0 and ry1==-1 and rz1==1 and self.rotation!=RotationActivity.REAR_UP:
                self.save_to_memory(self.conf_data.keys()[0], data=write_data)
                self.raise_event(self.conf_data.keys()[0], 'rear_up')

            elif rx1==-1 and ry0==0 and rz1==1 and self.rotation!=RotationActivity.RIGHT_UP:
                self.save_to_memory(self.conf_data.keys()[0], data=write_data)
                self.raise_event(self.conf_data.keys()[0], 'right_up')

            elif rx1==1 and ry0==0 and rz1==1 and self.rotation!=RotationActivity.LEFT_UP:
                self.save_to_memory(self.conf_data.keys()[0], data=write_data)
                self.raise_event(self.conf_data.keys()[0], 'left_up')

            elif rx0==0 and ry0==0 and rz1==2 and self.rotation!=RotationActivity.BOTTOM_UP:
                self.save_to_memory(self.conf_data.keys()[0], data=write_data)
                self.raise_event(self.conf_data.keys()[0], 'bottom_up')

            time.sleep(0.1)

    def stop(self):
        self._run = False


    def read_byte(self, dr):
        return self.bus.read_byte_data(self.address, adr)

    def read_word(self, adr):
        high = self.bus.read_byte_data(self.address, adr)
        low = self.bus.read_byte_data(self.address, adr+1)
        val = (high << 8) + low
        return val

    def read_word_2c(self, adr):
        val = self.read_word(adr)
        if (val >= 0x8000):
            return -((65535 - val) + 1)
        else:
            return val

    def dist(self, a,b):
        return math.sqrt((a*a)+(b*b))

    def get_y_rotation(self,x,y,z):
        radians = math.atan2(x, self.dist(y,z))
        return -math.degrees(radians)

    def get_x_rotation(self,x,y,z):
        radians = math.atan2(y, self.dist(x,z))
        return math.degrees(radians)

    def get_z_rotation(self,x,y,z):
        radians = math.atan2(self.dist(y,x), z)
        return math.degrees(radians)


if __name__ == '__main__':
    t = BoxyRotation()
    t.start()

    rospy.spin()

    t.stop()

    exit(0)
