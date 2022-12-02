#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import smach
import re
from navigation_pr2.utils import *

class Idling(smach.State):
    def __init__(self, client):
        smach.State.__init__(self, outcomes=['timeout', 'start navigation', 'start mapping', 'end', 'aborted', 'intro'])
        self.speak = client

    def execute(self, userdata):
        if not wait_for_speech(timeout=30):
            return 'timeout'
        speech_raw = rospy.get_param('~speech_raw').encode('utf-8')
        rospy.delete_param('~speech_raw')
        if re.search(r'.*案内.*$', speech_raw):
            return 'start navigation'
        elif re.search(r'.*覚え.*$', speech_raw):
           return 'start mapping'
        elif re.search(r'.*終了.*$', speech_raw):
            return 'end'
        elif re.search(r'.*こんにちは.*$', speech_raw):
            self.speak.say('こんにちは。私はPR2です。')
            return 'intro'
        else:
            self.speak.parrot(speech_raw)
            return 'aborted'

        return 'start mapping'

class Initialize(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
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
        smach.State.__init__(self, outcomes=['succeeded', 'timeout', 'aborted'],
                             output_keys=['person_name'])
        self.speak = client
    def execute(self, userdata):
        self.speak.say('あなたの名前を教えて下さい。')
        if not wait_for_speech(timeout=30):
            return 'timeout'
        speech_raw = rospy.get_param('~speech_raw').encode('utf-8')
        rospy.delete_param('~speech_raw')
        if re.search(r'(.*)です.*$', speech_raw) is not None:
            person_name = re.search(r'(.*)です.*$', speech_raw).group(1)
            userdata.person_name = person_name
            self.speak.say('{}さんですね。よろしくお願いします。'.format(person_name))
            return 'succeeded'
        return 'aborted'

# wait
# explain use
# explain available spots
