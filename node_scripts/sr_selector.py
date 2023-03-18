#!/usr/bin/env python

import os
import rospy
from topic_tools.srv import MuxSelect
from std_msgs.msg import Bool
from sound_play.libsoundplay import SoundClient

connect_sound = "/home/nakane/nakane_ws/src/navigation_pr2/sound/connect.wav"
disconnect_sound = "/home/nakane/nakane_ws/src/navigation_pr2/sound/disconnect.wav"

class SRSelector():
    def __init__(self):
        self.tries = rospy.get_param('~tries', 1)
        self.timeout = rospy.get_param('~timeout', 2)
        self.host = rospy.get_param('~host', '8.8.8.8')
        self.active_engine = "google"
        self.trigger = True
        self.connected = True
        self.mux_client = rospy.ServiceProxy('/input_sr_mux/select',MuxSelect)
        self.sound = SoundClient(sound_action='robotsound', blocking=False)
        self.sound.actionclient.wait_for_server()
        self.pub = rospy.Publisher('/ping', Bool, queue_size=1)
        self.sub = rospy.Subscriber('/speech_recognition_google/connection_status', Bool, self.status_cb)
        self.timer = rospy.Timer(rospy.Duration(1.0), self.timer_cb)
        
    def check(self, host):
        response = os.system("ping -c {} -w {} -q {}".format(self.tries, self.timeout, host))
        return response

    def status_cb(self, msg):
        if msg.data:
            self.connected = True
        else:
            self.connected = False

    def timer_cb(self, event):
        res = self.check(self.host)
        if res == 0:
            if self.trigger == True:
                rospy.loginfo('network connected')
                print(connect_sound)
                self.sound.playWave(connect_sound, replace=False)
                self.trigger = False
            if self.connected:
                self.switch("google")
            else:
                self.switch("vosk")
            self.pub.publish(Bool(data=True))
        else:
            if self.trigger == False:
                rospy.loginfo('network disconnected')
                self.sound.playWave(disconnect_sound, replace=False)
                self.trigger = True
                self.switch("vosk")
            self.pub.publish(Bool(data=False))

    def switch(self, engine):
        if self.active_engine == engine:
            return
        if engine == "google":
            mux_client("/Tablet/voice_stamped/google")
            rospy.loginfo("switch to google")
        if engine == "vosk":
            mux_client("/Tablet/voice_stamped/vosk_filtered")
            rospy.loginfo("switch to vosk")
        self.active_engine = engine
        
if __name__ == '__main__':
    rospy.init_node('sr_selector')
    s = SRSelector()
    rospy.spin()
