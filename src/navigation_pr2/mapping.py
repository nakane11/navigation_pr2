#!/usr/bin/env python

import smach
import re
import rospy
from navigation_pr2.utils import *

class WaitForTeaching(smach.State):
    def __init__(self, client):
        smach.State.__init__(self, outcomes=['timeout', 'name received', 'end', 'request navigation', 'aborted'],
                             output_keys=['new_spot_name'])
        self.speak = client

    def execute(self, userdata):
        if not wait_for_speech(timeout=30):
            return 'timeout'
        speech_roman = rospy.get_param('~speech_roman')
        rospy.delete_param('~speech_roman')
        if re.findall('kokoga|dayo', speech_roman):
            userdata.new_spot_name = re.search(r'^kokoga(.*)dayo$', speech_roman).groups()[0]
            return 'name received'
        elif re.findall('owari', speech_roman):
            return 'end'
        elif re.findall('annai', speech_roman):
            return 'request navigation'
        else:
            self.speak.parrot(speech_roman)
            return 'aborted'

class SendWithName(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['send spot with name'], input_keys=['new_spot_name'])

    def execute(self, userdata):
        name = userdata.new_spot_name
        rospy.loginfo("add new spot: {}".format(name))
        #send goal
        return 'send spot with name'

class SendWithoutName(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['send spot', 'preempted'])

    def execute(self, userdata):
        #send goal
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        rospy.sleep(10.0)
        return 'send spot'

class SetMapAvailable(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'],
                             output_keys=['map_available'])

    def execute(self, userdata):
        userdata.map_available = 'true'
        return 'succeeded'    

class ExplainMapping(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        # speak
        return 'succeeded'

def con_mapping_child_term_cb(outcome_map):
    if outcome_map['RECORD_WITH_NAME'] == 'succeeded':
        return True
    if outcome_map['RECORD_WITH_NAME'] == 'request navigation':
        return True
    return False

def con_mapping_out_cb(outcome_map):
    if outcome_map['RECORD_WITH_NAME'] == 'succeeded':
        return 'succeeded'
    if outcome_map['RECORD_WITH_NAME'] == 'request navigation':
        return 'start navigation'
    

