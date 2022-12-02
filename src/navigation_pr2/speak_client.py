#!/usr/bin/env python
# -*- coding: utf-8 -*-
from std_msgs.msg import String
import rospy

class SpeakClient():
    def __init__(self, server_name='/speak_node'):
        self.topic_name = server_name + '/say'
        self.volume_name = server_name + '/volume'
        self.pub = rospy.Publisher(self.topic_name, String, queue_size=1)

    def say(self, text):
        str = String(data=text)
        self.pub.publish(str)

    def parrot(self, text):
        script = "{}ですか？ よくわかりませんでした。"
        self.say(script)

    def get_volume(self):
        volume =rospy.get_param(self.volume_name)
        return volume

    def set_volume(self, volume):
        volume =rospy.set_param(self.volume_name, volume)
        rospy.loginfo('set volume to {}'.format(volume))
