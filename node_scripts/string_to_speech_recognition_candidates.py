#!/usr/bin/env python

from speech_recognition_msgs.msg import SpeechRecognitionCandidates
import rospy
from jsk_topic_tools import ConnectionBasedTransport
from std_msgs.msg import String

class StringToSpeechRecognitionCandidates(ConnectionBasedTransport):

    def __init__(self):
        super(StringToSpeechRecognitionCandidates, self).__init__()
        self.pub = self.advertise("~output", SpeechRecognitionCandidates, queue_size=1)
                    
    def subscribe(self):
        self.sub = rospy.Subscriber('~input', String, self._cb)

    def unsubscribe(self):
        self.sub.unregister()

    def _cb(self, msg):
        text = msg.data
        self.pub.publish(SpeechRecognitionCandidates(transcript = [data]))
        rospy.loginfo("text: {}".format(text))

if __name__ == '__main__':
    rospy.init_node('string_to_speech_recognition_candidates')
    StringToSpeechRecognitionCandidates()
    rospy.spin()
