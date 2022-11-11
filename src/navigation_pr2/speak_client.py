#!/usr/bin/env python
# -*- coding: utf-8 -*-
from std_msgs.msg import String

class SpeakClient():
    def __init__(self, server_name='/speak_node'):
        self.topic_name = server_name + '/say'
        self.volume_name = server_name + '/volume'
        self.pub = rospy.Publisher(self.topic_name, String)

    def say(self, text):
        str = String(data=text)
        self.pub.publish(str)

    def parrot(self, text):
        script = "{}ですか？ よくわかりませんでした。".format(text)
        self.say(script)

    def get_volume(self):
        volume =rospy.get_param(self.volume_name)
        return volume
