#!/usr/bin/env python

import os
import rospy
from std_msgs.msg import Bool

class Ping():
    def __init__(self):
        self.tries = rospy.get_param('~tries', 1)
        self.timeout = rospy.get_param('~timeout', 2)
        self.host = rospy.get_param('~host', '8.8.8.8')
        self.pub = rospy.Publisher('/ping', Bool, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1.0), self.timer_cb)
        
    def check(self, host):
        response = os.system("ping -c {} -w {} -q {}".format(self.tries, self.timeout, host))
        return response

    def timer_cb(self, event):
        res = self.check(self.host)
        if res == 0:
            self.pub.publish(Bool(data=True))
        else:
            self.pub.publish(Bool(data=False))

if __name__ == '__main__':
    rospy.init_node('ping')
    p = Ping()
    rospy.spin()
