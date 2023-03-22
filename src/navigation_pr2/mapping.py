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
from navigation_pr2.msg import ChangeFloorAction, ChangeFloorGoal
from virtual_force_drag.msg import SwitchAction, SwitchGoal

class WaitForTeaching(smach.State):
    def __init__(self, client):
        smach.State.__init__(self, outcomes=['timeout', 'name received',\
                                             'description received', 'end',\
                                             'request navigation', 'aborted', 'cancelled'],
                             output_keys=['new_spot_name', 'new_description'])
        self.multiple_floor = rospy.get_param('~multiple_floor', True)
        self.speak = client
        rospy.loginfo('waiting for spot_map_server/change_floor...')
        rospy.wait_for_service('spot_map_server/change_floor')
        self.eus_floor = rospy.ServiceProxy('/spot_map_server/change_floor', ChangeFloor)
        rospy.loginfo('waiting for spot_map_server/stop...')
        rospy.wait_for_service('spot_map_server/stop')
        self.stop = rospy.ServiceProxy('spot_map_server/stop', Empty)

        if self.multiple_floor:
            self.ac = actionlib.SimpleActionClient('map_manager/change_floor', ChangeFloorAction)
            rospy.loginfo('waiting for map_manager/change_floor...')
            self.ac.wait_for_server()
            goal = ChangeFloorGoal()
            goal.command = 0
            goal.floor = 'empty'
            self.ac.send_goal(goal)
            rospy.loginfo('waiting for change_floor result...')
            self.ac.wait_for_result()

        self.initialized = False
        self.last_spot_name = ''

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
                        if self.multiple_floor:
                            self.speak.say('地図を作るのでそのままお待ち下さい')
                            goal = ChangeFloorGoal()
                            goal.command = 1
                            goal.floor = floor_name
                            self.ac.send_goal(goal)
                            rospy.loginfo('waiting for change_floor result...')
                            self.ac.wait_for_result()
                            self.speak.say('準備完了です')
                        rospy.set_param('~floor', floor_name)
                        break
                else:
                    continue
            self.initialized = True
            return 'aborted'

        if not wait_for_speech(timeout=120):
            return 'timeout'
        speech_raw = rospy.get_param('~speech_raw').encode('utf-8')
        rospy.delete_param('~speech_raw')
        if re.search(r'^(.*)階.*到着.*$', speech_raw) is not None:
            self.floor_name = re.search(r'^(.*)階.*$', speech_raw).group(1)
            self.speak.say('{}階ですね。ちょっと待ってください'.format(self.floor_name))
            floor_name = floors[self.floor_name]
            self.eus_floor(floor=floor_name)
            if self.multiple_floor:
                goal = ChangeFloorGoal()
                goal.command = 1
                goal.floor = floor_name
                self.ac.send_goal(goal)
                rospy.loginfo('waiting for change_floor result...')
                self.ac.wait_for_result()
                self.speak.say('お待たせしました。{}階の地図を用意しました'.format(self.floor_name))
                # self.py_floor(command=1, floor=floor_name)
            rospy.set_param('~floor', floor_name)
            return 'aborted'
        if re.search(r'.*ここが(.*)だよ$', speech_raw) is not None:
            userdata.new_spot_name = re.search(r'ここが(.*)だよ.*$', speech_raw).group(1)
            self.last_spot_name = re.search(r'ここが(.*)だよ.*$', speech_raw).group(1)
            self.speak.say('{}というのですね'.format(re.search(r'ここが(.*)だよ.*$', speech_raw).group(1)))
            return 'name received'
        if self.last_spot_name is not '':
            if re.search(r'.*{}は(.*)$'.format(self.last_spot_name), speech_raw) is not None:
                userdata.new_description = re.search(r'.*{}は(.*)$'.format(self.last_spot_name), speech_raw).group(1)
                self.speak.say('そうなのですね')
                return 'description received'
        if re.search(r'.*違い.*$', speech_raw) is not None:
            self.speak.say('失礼しました')
            return 'cancelled'
        if re.findall('終了', speech_raw):
            print(self.floor_name)
            floor_name = floors[self.floor_name]
            self.stop()
            if self.multiple_floor:
                goal = ChangeFloorGoal()
                goal.command = 2
                goal.floor = floor_name
                self.ac.send_goal(goal)
                rospy.loginfo('waiting for change_floor result...')
                self.ac.wait_for_result()
                # self.py_floor(command=2, floor=floor_name)
            self.speak.say('はい')
            return 'end'
        if re.findall('案内', speech_raw):
            return 'request navigation'
        self.speak.parrot(speech_raw)
        return 'aborted'

class StartMapping(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.debug = rospy.get_param('~debug', False)
        self.ac = actionlib.SimpleActionClient('/lead_pr2_action', SwitchAction)
        rospy.loginfo('waiting for lead_pr2_action...')
        self.ac.wait_for_server()

    def execute(self, userdata):
        if self.debug:
            rospy.loginfo('skipped switch goal')
            return 'succeeded'
        goal = SwitchGoal(switch=True)
        self.ac.send_goal(goal)
        return 'succeeded'

class FinishMapping(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.debug = rospy.get_param('~debug', False)
        self.ac = actionlib.SimpleActionClient('/lead_pr2_action', SwitchAction)
        rospy.loginfo('waiting for lead_pr2_action...')
        self.ac.wait_for_server()

    def execute(self, userdata):
        if self.debug:
            rospy.loginfo('skipped switch goal')
            return 'succeeded'
        goal = SwitchGoal(switch=False)
        self.ac.send_goal(goal)
        return 'succeeded'

class SendWithName(smach.State):
    def __init__(self, client):
        smach.State.__init__(self, outcomes=['send spot with name', 'register elevator'],
                             input_keys=['new_spot_name'])
        self.ac = actionlib.SimpleActionClient('/spot_map_server/add', RecordSpotAction)
        rospy.loginfo('waiting for spot_map_server/add...')
        self.ac.wait_for_server()
        self.speak = client

    def execute(self, userdata):
        name = userdata.new_spot_name
        rospy.loginfo("add new spot: {}".format(name))
        if re.search(r'.*エ.*ベータ.*$', name) is not None:
            print("sendwithname elevator")
            goal = RecordSpotGoal()
            goal.command = 3
            goal.name = name
            goal.node.type = 1
            goal.update_keys = ['type']
            self.ac.send_goal(goal)
            self.ac.wait_for_result()
            result = self.ac.get_result().result
            flag = True
            while result is False:
                if flag:
                    self.speak.say('位置を確認するので待って下さい')
                    flag = False
                self.ac.send_goal(goal)
                self.ac.wait_for_result()
                print(self.ac.get_result())
                result = self.ac.get_result().result
                rospy.loginfo('wait for transform')
                if result:
                    self.speak.say('記録しました')
            return 'register elevator'

        goal = RecordSpotGoal()
        goal.command = 1
        goal.name = name
        self.ac.send_goal(goal)
        self.ac.wait_for_result()
        result = self.ac.get_result().result
        flag = True
        while result is False:
            if flag:
                self.speak.say('位置を確認するので待って下さい')
                flag = False
            self.ac.send_goal(goal)
            self.ac.wait_for_result()
            print(self.ac.get_result())
            result = self.ac.get_result().result
            rospy.loginfo('wait for transform')
            if result:
                self.speak.say('記録しました')
        return 'send spot with name'

class SendDescription(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'], input_keys=['new_spot_name', 'new_description'])
        self.ac = actionlib.SimpleActionClient('/spot_map_server/add', RecordSpotAction)
        rospy.loginfo('waiting for spot_map_server/add...')
        self.ac.wait_for_server()

    def execute(self, userdata):
        name = userdata.new_spot_name
        description = userdata.new_description
        rospy.loginfo("add new description to {}: {}".format(name, description))
        goal = RecordSpotGoal()
        goal.command = 4
        goal.name = name
        goal.node.description = description
        goal.update_keys = ['description']
        self.ac.send_goal(goal)
        return 'succeeded'

class SendCancelName(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.ac = actionlib.SimpleActionClient('/spot_map_server/add', RecordSpotAction)
        rospy.loginfo('waiting for spot_map_server/add...')
        self.ac.wait_for_server()

    def execute(self, userdata):
        goal = RecordSpotGoal()
        goal.command = 2
        self.ac.send_goal(goal)
        return 'succeeded'

class SwitchRecordWithoutName(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['preempted'])
        rospy.loginfo('waiting for spot_map_server/start...')
        rospy.wait_for_service('spot_map_server/start')
        self.start = rospy.ServiceProxy('spot_map_server/start', Empty)
        rospy.loginfo('waiting for spot_map_server/stop...')
        rospy.wait_for_service('spot_map_server/stop')
        self.stop = rospy.ServiceProxy('spot_map_server/stop', Empty)

    def execute(self, userdata):
        self.start()
        while not rospy.is_shutdown():
            if self.preempt_requested():
                self.stop()
                self.service_preempt()
                return 'preempted'
            rospy.sleep(1.0)

class ExplainMapping(smach.State):
    def __init__(self, client):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.speak = client
    def execute(self, userdata):
        # speak
        self.speak.say('お困りですか？「ここが研究室だよ」というように場所の名前を教えて下さい。')
        return 'succeeded'

def con_mapping_child_term_cb(outcome_map):
    if outcome_map['RECORD_WITH_NAME'] == 'succeeded':
        return True
    if outcome_map['RECORD_WITH_NAME'] == 'request navigation':
        return True
    if outcome_map['RECORD_WITH_NAME'] == 'elevator':
        return True
    return False

def con_mapping_out_cb(outcome_map):
    if outcome_map['RECORD_WITH_NAME'] == 'succeeded':
        return 'succeeded'
    if outcome_map['RECORD_WITH_NAME'] == 'request navigation':
        return 'start navigation'
    if outcome_map['RECORD_WITH_NAME'] == 'elevator':
        return 'elevator'
