#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from speech_recognition_msgs.msg import SpeechRecognitionCandidates, SpeechRecognitionCandidatesStamped

class FilterVosk(object):
    def __init__(self):
        self.pub = rospy.Publisher("/Tablet/voice/filtered", SpeechRecognitionCandidates, queue_size=1)
        self.pub_stamped = rospy.Publisher("/Tablet/voice_stamped/filtered", SpeechRecognitionCandidatesStamped, queue_size=1)
        self.sub = rospy.Subscriber("/Tablet/voice", SpeechRecognitionCandidates, self.cb, queue_size=1)
        self.sub_stamped = rospy.Subscriber("/Tablet/voice_stamped", SpeechRecognitionCandidatesStamped, self.cb_stamped, queue_size=1)
        self.black_list = ["あー", "えー", "ん"]
        
    def cb(self, msg):
        for word in self.black_list:
            if msg.transcript[0] == word: return
        msg.transcript[0] = msg.transcript[0].replace(' ', '')
        self.pub.publish(msg) 

    def cb_stamped(self, msg):
        for word in self.black_list:
            if msg.candidates.transcript[0] == word: return
        msg.candidates.transcript[0] = msg.candidates.transcript[0].replace(' ', '')
        self.pub_stamped.publish(msg)        
        return

if __name__ == "__main__":
    rospy.init_node('filter_vosk')
    f = FilterVosk()
    rospy.spin()

                        
