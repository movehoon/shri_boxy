#!/usr/bin/env python
#-*- encoding: utf8 -*-

import sys
from datetime import datetime
import os
import rospy
from mhri_msgs.msg import LogItem


class LoggerExample:
    def __init__(self):
        rospy.init_node('boxy_logger_node', anonymous=False)

        try:
            path_param = rospy.get_param('~log_path')
            self.log_path = os.path.expanduser(path_param)
        except KeyError as e:
            rospy.logerr(rospy.get_name() + ': %s'%e)
            quit()

        if not os.path.exists(self.log_path):
            rospy.loginfo('{0} did not exist; creating it.'.format(self.log_path))
            os.makedirs(self.log_path)

        print self.log_path
        self.log_file = open(self.log_path + '/log_%s.txt'%str(datetime.now()), 'w')
        rospy.Subscriber('/log', LogItem, self.handle_log_item)

        rospy.loginfo("\033[91m[%s]\033[0m initialized." % rospy.get_name())

    def handle_log_item(self, msg):
        time_header = msg.header.stamp

        log_content = '%s_'%str(datetime.now().date())
        log_content += '%s : '%time_header
        for item in msg.log_items:
            log_content += item + '\t'

        log_content += '\r\n'
        self.log_file.write(log_content)

if __name__ == '__main__':
    m = LoggerExample()
    rospy.spin()
