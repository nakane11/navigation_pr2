#!/usr/bin/env python
# -*- coding: utf-8 -*-
from speech_recognition_msgs.msg import SpeechRecognitionCandidates
from navigation_pr2 import SpeakClient
import rospy
import re

class DialogueNode():
    def __init__(self):
        self.speech_sub = rospy.Subscriber('/Tablet/voice', SpeechRecognitionCandidates, self.speech_cb)
        self.speak = SpeakClient()

    def speech_cb(self, msg):
        result = self.execute(msg.transcript[0], msg.transcript[1])
        if not result:
            self.speak.parrot(msg.transcript[0])

    def execute(self, text_raw, text_roman):
        if self.next_function_existed:
            self.next_function_existed = False
            self.next_function(text_raw, text_roman)

        if re.findall('basyo|oboe', text_roman):
            self.speak.say("場所を覚えます。腕を引いて移動して下さい")
            self.status = 'make_map'
            # start teach spot
            # start virtual_force_drag
            return

        if re.findall('kokoga|dayo', text_roman):
            place_name = re.search(r'^kokoga(.*)dayo$', text_roman).groups()[0]
            if self.status == 'make_map':
                add_spot(place_name)
            else:
                self.speak.say('この場所を覚えますか')
                self.set_next_func("")
            return

        if re.findall('owari', text_roman):
            self.end_request()

        if re.findall('tomat', text_roman):
            self.inturrupt_request()

        return False

    def end_request(self):
        if self.status == 'make_map':
            end_make_map()
            # stop teach_spot
            # stop virtual_force_drag
        elif self.status == 'navigation':
            end_navigation()

    def inturrupt_request(self):
        if self.status == 'make_map':
            end_make_map()
            # stop teach_spot
            # stop virtual_force_drag
        elif self.status == 'navigation':
            end_navigation()

    def set_next_func(self, f):
        self.next_function = f
        self.next_function_existed = True

if __name__ == '__main__':
    rospy.init_node('dialogue')
    dialogue = DialogueNode()
    rospy.spin()
