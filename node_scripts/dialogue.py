#!/usr/bin/env python
# -*- coding: utf-8 -*-
from speech_recognition_msgs.msg import SpeechRecognitionCandidates
from navigation_pr2 import SpeakClient
import rospy

class DialogueNode():
    def __init__(self):
        self.speech_sub = rospy.Subscriber('/Tablet/voice', SpeechRecognitionCandidates, self.speech_cb)
        self.speak = SpeakClient()

    def speech_cb(self, msg):
        result = self.execute(msg.transcript[0], msg.transcript[1])
        if not result:
            self.speak.parrot(msg.transcript[0])

    def execute(self, text_raw, text_roman):
        return False

if __name__ == '__main__':
    rospy.init_node('dialogue')
    dialogue = DialogueNode()
    rospy.spin()
