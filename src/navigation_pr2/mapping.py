#!/usr/bin/env python
# -*- coding: utf-8 -*-

import smach
import re
import rospy
import actionlib
from std_srvs.srv import Empty
from navigation_pr2.utils import *
from navigation_pr2.msg import RecordSpotAction, RecordSpotGoal
from navigation_pr2.srv import ChangeFloor

floors = {'一':'1', '二':'2', '三':'3', '四':'4', '五':'5', '六':'6', '七':'7', '八':'8', '九':'9'}

class WaitForTeaching(smach.State):
    def __init__(self, client):
        smach.State.__init__(self, outcomes=['timeout', 'name received', 'end', 'request navigation', 'aborted', 'cancelled'],
                             output_keys=['new_spot_name'])
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
                self.speak.say('ここは何階ですか?')
                rospy.loginfo('waiting for floor...')
                if wait_for_speech(timeout=20):
                    speech_raw = rospy.get_param('~speech_raw').encode('utf-8')
                    rospy.delete_param('~speech_raw')
                    if re.search(r'(.*)階.*$', speech_raw) is not None:
                        self.floor_name = re.search(r'(.*)階.*$', speech_raw).group(1)
                        self.speak.say('{}階ですね。ありがとうございます'.format(self.floor_name))
                        floor_name = floors[self.floor_name]
                        self.eus_floor(floor=floor_name)
                        self.py_floor(command=1, floor=floor_name)
                        break
                else:
                    continue
            self.initialized = True
            return 'aborted'

        if not wait_for_speech(timeout=30):
            return 'timeout'
        speech_raw = rospy.get_param('~speech_raw').encode('utf-8')
        rospy.delete_param('~speech_raw')
        if re.search(r'.*ここが(.*)だよ$', speech_raw) is not None:
            userdata.new_spot_name = re.search(r'ここが(.*)だよ.*$', speech_raw).group(1)
            self.speak.say('{}というのですね'.format(re.search(r'ここが(.*)だよ.*$', speech_raw).group(1)))
            return 'name received'
        elif re.search(r'.*違い.*$', speech_raw) is not None:
            self.speak.say('失礼しました')
            return 'cancelled'
        elif re.search(r'^(.*)階.*到着.*$', speech_raw) is not None:
            self.floor_name = re.search(r'^(.*)階.*$', speech_raw).group(1)
            self.speak.say('{}階ですね。ちょっと待ってください'.format(self.floor_name))
            floor_name = floors[self.floor_name]
            self.eus_floor(floor=floor_name)
            self.py_floor(command=1, floor=floor_name)
            return 'aborted'
        elif re.findall('終了', speech_raw):
            print(self.floor_name)
            floor_name = floors[self.floor_name]
            self.py_floor(command=2, floor=floor_name)
            return 'end'
        elif re.findall('案内', speech_raw):
            return 'request navigation'
        self.speak.parrot(speech_raw)
        return 'aborted'

class SendWithName(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['send spot with name'], input_keys=['new_spot_name'])
        self.ac = actionlib.SimpleActionClient('/record_spot', RecordSpotAction)
        self.ac.wait_for_server()

    def execute(self, userdata):
        name = userdata.new_spot_name
        rospy.loginfo("add new spot: {}".format(name))
        goal = RecordSpotGoal()
        goal.command = 1
        goal.name = name
        self.ac.send_goal(goal)
        return 'send spot with name'

class SendCancelName(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.ac = actionlib.SimpleActionClient('/record_spot', RecordSpotAction)
        self.ac.wait_for_server()

    def execute(self, userdata):
        goal = RecordSpotGoal()
        goal.command = 2
        self.ac.send_goal(goal)
        return 'succeeded'

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
