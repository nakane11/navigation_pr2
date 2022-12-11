#!/usr/bin/env python
# -*- coding: utf-8 -*-
from std_msgs.msg import String
import rospy
from navigation_pr2.msg import SpeakAction, SpeakGoal
import actionlib

class SpeakClient():
    def __init__(self, server_name='/speak_node'):
        self.topic_name = server_name + '/say'
        self.volume_name = server_name + '/volume'
        self.pub = rospy.Publisher(self.topic_name, String, queue_size=1)
        self.ac = actionlib.SimpleActionClient('/speak_node/say', SpeakAction)

    def say(self, text):
        # str = String(data=text)
        goal = SpeakGoal(data=text)
        self.ac.send_goal(goal)
        self.ac.wait_for_result()
        # self.pub.publish(str)

    def parrot(self, text):
        script = "{}ですか？ よくわかりませんでした。".format(text)
        self.say(script)

    def get_volume(self):
        volume =rospy.get_param(self.volume_name)
        return volume

    def set_volume(self, volume):
        volume =rospy.set_param(self.volume_name, volume)
        rospy.loginfo('set volume to {}'.format(volume))
