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
from navigation_pr2.srv import ChangeFloor

class WaitForTeaching(smach.State):
    def __init__(self, client):
        smach.State.__init__(self, outcomes=['timeout', 'name received', 'end', 'request navigation', 'aborted'],
                             output_keys=['new_spot_name', 'new_spot_name_jp'])
        self.speak = client
        rospy.wait_for_service('/spot_map_server/change_floor')
        rospy.wait_for_service('/map_manager/change_floor')
        self.eus_floor = rospy.ServiceProxy('/spot_map_server/change_floor', ChangeFloor)
        self.py_floor = rospy.ServiceProxy('/map_manager/change_floor', ChangeFloor)
        self.py_floor(command=0, floor='empty')
        self.initialized = False

    def execute(self, userdata):
        if not self.initialized:
            while True:
                self.speak.say('ここは何階ですか')
                rospy.loginfo('waiting for floor...')
                if wait_for_speech(timeout=20):
                    speech_roman = rospy.get_param('~speech_roman')
                    rospy.delete_param('~speech_raw')
                    rospy.delete_param('~speech_roman')
                    if re.findall('kai', speech_roman):
                    if re.search(r'(.*)kai.*$', speech_roman) is not None:
                        floor_name = re.search(r'(.*)kai.*$', speech_roman).group(1)
                        self.speak.say('{}階ですね'.format(romkan.to_hiragana(floor_name).encode('utf-8')))
                        self.eus_floor(floor=floor_name)
                        self.py_floor(command=1, floor=floor_name)
                        break
                else:
                    continue
            self.initialized = True
            return 'aborted'

        if not wait_for_speech(timeout=30):
            return 'timeout'
        speech_raw = rospy.get_param('~speech_raw')
        speech_roman = rospy.get_param('~speech_roman')
        rospy.delete_param('~speech_raw')
        rospy.delete_param('~speech_roman')
        if re.search(r'.*kokoga(.*)dayo$', speech_roman) is not None:
            extracted_name = re.search(r'^kokoga(.*)dayo$', speech_roman).group(1)
            userdata.new_spot_name = extracted_name
            userdata.new_spot_name_jp = romkan.to_hiragana(extracted_name)
            self.speak.say('{}というのですね'.format(romkan.to_hiragana(extracted_name).encode('utf-8')))
            return 'name received'
        elif re.search(r'^(.*)kai.*tuita.*$', speech_roman) is not None:
            floor_name = re.search(r'^(.*)kai.*tuita.*$', speech_roman).group(1)
            self.speak.say('{}階ですね'.format(romkan.to_hiragana(floor_name).encode('utf-8')))
            self.eus_floor(floor=floor_name)
            self.py_floor(command=1, floor=floor_name)
            return 'aborted'
        elif re.findall('owari', speech_roman):
            self.py_floor(command=2, floor=floor_name)
            return 'end'
        elif re.findall('annai', speech_roman):
            return 'request navigation'
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
