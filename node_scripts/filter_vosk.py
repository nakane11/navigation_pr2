#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from speech_recognition_msgs.msg import SpeechRecognitionCandidates, SpeechRecognitionCandidatesStamped

class FilterVosk(object):
    def __init__(self):
        self.pub = rospy.Publisher("/Tablet/voice/vosk_filtered", SpeechRecognitionCandidates, queue_size=1)
        self.pub_stamped = rospy.Publisher("/Tablet/voice_stamped/vosk_filtered", SpeechRecognitionCandidatesStamped, queue_size=1)
        self.sub = rospy.Subscriber("/Tablet/voice/vosk", SpeechRecognitionCandidates, self.cb, queue_size=1)
        self.sub_stamped = rospy.Subscriber("/Tablet/voice_stamped/vosk", SpeechRecognitionCandidatesStamped, self.cb_stamped, queue_size=1)
        self.black_list = ["あー", "えー", "ん", "あ", "うん", "えーん", "んん", "", "ああ", "う", "なぁ", "え"]
        
    def cb(self, msg):
        msg.transcript[0] = msg.transcript[0].replace(' ', '')
        msg.transcript[0] = msg.transcript[0].strip('ん')
        for word in self.black_list:
            if msg.transcript[0] == word: return
        self.pub.publish(msg) 

    def cb_stamped(self, msg):
        print(type(msg.candidates.transcript[0]))
        text = msg.candidates.transcript[0].replace(' ', '')
        text = text.strip('ん')
        text = text.strip('あー')
        for word in self.black_list:
            if text == word: return
        msg.candidates.transcript[0] = text
        self.pub_stamped.publish(msg)        
        return

if __name__ == "__main__":
    rospy.init_node('filter_vosk')
    f = FilterVosk()
    rospy.spin()

                        
