#!/usr/bin/env python
# -*- coding: utf-8 -*-

import smach
import re
import rospy
import actionlib
import romkan
from std_srvs.srv import Empty
from navigation_pr2.utils import *
from navigation_pr2.msg import RecordSpotAction, RecordSpotGoal

class WaitForTeaching(smach.State):
    def __init__(self, client):
        smach.State.__init__(self, outcomes=['timeout', 'name received', 'end', 'request navigation', 'aborted'],
                             output_keys=['new_spot_name', 'new_spot_name_jp'])
        self.speak = client

    def execute(self, userdata):
        if not wait_for_speech(timeout=30):
            return 'timeout'
        speech_raw = rospy.get_param('~speech_raw')
        speech_roman = rospy.get_param('~speech_roman')
        rospy.delete_param('~speech_raw')
        rospy.delete_param('~speech_roman')
        if re.findall('kokoga|dayo', speech_roman):
            extracted_name = re.search(r'^kokoga(.*)dayo$', speech_roman).groups()[0]
            userdata.new_spot_name = extracted_name
            # userdata.new_spot_name_jp = re.search(r'^ここが(.*)だよ$', speech_raw.encode('utf-8')).groups()[0]
            userdata.new_spot_name_jp = romkan.to_hiragana(extracted_name)
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
        smach.State.__init__(self, outcomes=['send spot with name'], input_keys=['new_spot_name', 'new_spot_name_jp'])
        self.ac = actionlib.SimpleActionClient('/record_spot', RecordSpotAction)
        self.ac.wait_for_server()

    def execute(self, userdata):
        name = userdata.new_spot_name
        name_jp = userdata.new_spot_name_jp
        rospy.loginfo("add new spot: {}".format(name_jp.encode('utf-8')))
        goal = RecordSpotGoal()
        goal.command = 1
        goal.name = name
        goal.name_jp = name_jp
        self.ac.send_goal(goal)
        return 'send spot with name'

class SwitchRecordWithoutName(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['preempted'])
        rospy.wait_for_service('start_auto_map')
        self.start = rospy.ServiceProxy('start_auto_map', Empty)
        rospy.wait_for_service('stop_auto_map')
        self.stop = rospy.ServiceProxy('stop_auto_map', Empty)

    def execute(self, userdata):
        self.start()
        while not rospy.is_shutdown():
            if self.preempt_requested():
                self.stop()
                self.service_preempt()
                return 'preempted'
            rospy.sleep(1.0)

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
    

