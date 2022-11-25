#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import smach
import re
from navigation_pr2.utils import *

class Idling(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['timeout', 'start navigation', 'start mapping', 'end', 'aborted', 'intro'])

    def execute(self, userdata):
        if not wait_for_speech(timeout=30):
            return 'timeout'
        speech_roman = rospy.get_param('~speech_roman')
        rospy.delete_param('~speech_roman')
        if re.findall('annai', speech_roman):
            return 'start navigation'
        elif re.findall('oboe', speech_roman):
            return 'start mapping'
        elif re.findall('owari', speech_roman):
            return 'end'
        elif re.findall('konniti', speech_roman):
            return 'intro'
        else:
            # self.speak.parrot(speech_roman)
            return 'aborted'

        return 'start mapping'

class Initialize(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'],
                             output_keys=['map_available'])

    def execute(self, userdata):
        userdata.map_available = False
        return 'succeeded'

class Explain(smach.State):
    def __init__(self, client):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.speak = client
    def execute(self, userdata):
        self.speak.say('案内を開始するためには「案内」と話しかけて下さい')
        return 'succeeded'

class Introduction(smach.State):
    def __init__(self, client):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.speak = client
    def execute(self, userdata):
        self.speak.say('こんにちは。')
        return 'succeeded'

# wait
# explain use
# explain available spots
