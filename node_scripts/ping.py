#!/usr/bin/env python

import os
import rospy
from std_msgs.msg import Bool

from sound_play.libsoundplay import SoundClient

connect_sound = "/home/nakane/nakane_ws/src/navigation_pr2/sound/connect.wav"
disconnect_sound = "/home/nakane/nakane_ws/src/navigation_pr2/sound/disconnect.wav"

class Ping():
    def __init__(self):
        self.tries = rospy.get_param('~tries', 1)
        self.timeout = rospy.get_param('~timeout', 2)
        self.host = rospy.get_param('~host', '8.8.8.8')
        self.pub = rospy.Publisher('/ping', Bool, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1.0), self.timer_cb)
        self.trigger = True
        self.sound = SoundClient(sound_action='robotsound', blocking=False)
        self.sound.actionclient.wait_for_server()
        
    def check(self, host):
        response = os.system("ping -c {} -w {} -q {}".format(self.tries, self.timeout, host))
        return response

    def timer_cb(self, event):
        res = self.check(self.host)
        if res == 0:
            if self.trigger == True:
                rospy.loginfo('network connected')
                print(connect_sound)
                self.sound.playWave(connect_sound, replace=False)
                self.trigger = False
            self.pub.publish(Bool(data=True))
        else:
            if self.trigger == False:
                rospy.loginfo('network disconnected')
                self.sound.playWave(disconnect_sound, replace=False)
                self.trigger = True
            self.pub.publish(Bool(data=False))

if __name__ == '__main__':
    rospy.init_node('ping')
    p = Ping()
    rospy.spin()
