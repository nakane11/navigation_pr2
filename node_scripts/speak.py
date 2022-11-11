#!/usr/bin/env python
from sound_play.libsoundplay import SoundClient
import rospy

class SpeakNode(object):
    def __init__(self):
        self.volume = 1.0
        self.lang = rospy.get_param('~lang', 'jp')
        if self.lang == 'jp':
            self.sound_client = SoundClient(blocking=True, sound_action='robotsound_jp', sound_topic='robotsound_jp')
        elif self.lang == 'en':
            self.sound_client = SoundClient(blocking=True, sound_action='robotsound', sound_topic='robotsound')
        else:
            rospy.logerr("{} is not supported".format(self.lang))
            return
        self.sub = rospy.Subscriber("say", String, self.say)

    def say(self, msg):
        self.update_volume()
        self.sound_client.say(text, volume=vol)

    def update_volume(self):
        rospy.get_param('~volume', 1.0)
        if volume > 1.0 or volume < 0.0:
            rospy.loginfo("Volume is out of range. Do nothing.")
        else:
            self.volume = volume

if __name__ == '__main__':
    rospy.init_node('speak_node')
    speak = SpeakNode()
    rospy.spin()



        

    
             
            
