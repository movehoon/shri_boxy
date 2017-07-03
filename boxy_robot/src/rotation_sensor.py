#!/usr/bin/python

import os
import rospy

from threading import Thread
import time

from perception_base.perception_base import PerceptionBase

import smbus
import math

# Power management registers
power_mgmt_1 = 0x6b
power_mgmt_2 = 0x6c

TOP_UP = 0
FRONT_UP = 1
REAR_UP = 2
RIGHT_UP = 3
LEFT_UP = 4
BOTTOM_UP = 5

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

        self.rotation = TOP_UP
        self.stable_count = 0
        self.bak_x = 0
        self.bak_y = 0
        self.bak_z = 0

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

#            print ('%0.1f, %0.1f, %0.1f' %(rot_x, rot_y, rot_z))

            rx = -100
            if int(rot_x/20) == 0:
                rx = int(rot_x/20)
            if int(rot_x/70) != 0:
                rx = int(rot_x/70)

            ry = -100
            if int(rot_y/20) == 0:
                ry = int(rot_y/20)
            if int(rot_y/70) != 0:
                ry = int(rot_y/70)

            rz = -100
            if int(rot_z/20) == 0:
                rz = int(rot_z/20)
            if int(rot_z/70) != 0:
                rz = int(rot_z/70)

#            print ('rx=%d, ry=%d, rz=%d' %(rx, ry, rz))

            if rx == self.bak_x and ry == self.bak_y and rz == self.bak_z:
                self.stable_count = self.stable_count+1

                if self.stable_count > 5:
                    self.stable_count = 0
                    write_data = self.conf_data['rotation_activity']['data']
                    write_data['rotation_x'] = rot_x
                    write_data['rotation_y'] = rot_y
                    write_data['rotation_z'] = rot_z

                    if rx==0 and ry==0 and rz==0 and self.rotation!=TOP_UP:
                        self.rotation = TOP_UP
                        self.save_to_memory(self.conf_data.keys()[0], data=write_data)
                        self.raise_event(self.conf_data.keys()[0], 'top_up')

                    elif rx==0 and ry==1 and rz==1 and self.rotation!=FRONT_UP:
                        self.rotation = FRONT_UP
                        self.save_to_memory(self.conf_data.keys()[0], data=write_data)
                        self.raise_event(self.conf_data.keys()[0], 'front_up')

                    elif rx==0 and ry==-1 and rz==1 and self.rotation!=REAR_UP:
                        self.rotation = REAR_UP
                        self.save_to_memory(self.conf_data.keys()[0], data=write_data)
                        self.raise_event(self.conf_data.keys()[0], 'rear_up')

                    elif rx==1 and ry==0 and rz==1 and self.rotation!=RIGHT_UP:
                        self.rotation = RIGHT_UP
                        self.save_to_memory(self.conf_data.keys()[0], data=write_data)
                        self.raise_event(self.conf_data.keys()[0], 'right_up')

                    elif rx==-1 and ry==0 and rz==1 and self.rotation!=LEFT_UP:
                        self.rotation = LEFT_UP
                        self.save_to_memory(self.conf_data.keys()[0], data=write_data)
                        self.raise_event(self.conf_data.keys()[0], 'left_up')

                    elif rx==0 and ry==0 and rz==2 and self.rotation!=BOTTOM_UP:
                        self.rotation = BOTTOM_UP
                        self.save_to_memory(self.conf_data.keys()[0], data=write_data)
                        self.raise_event(self.conf_data.keys()[0], 'bottom_up')

            else:
                self.stable_count = 0

            self.bak_x = rx
            self.bak_y = ry
            self.bak_z = rz

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
