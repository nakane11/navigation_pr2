#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import smach
import re
import actionlib
import dynamic_reconfigure.client
from navigation_pr2.utils import *
from navigation_pr2.srv import ListSpotName
from navigation_pr2.srv import Path, ChangeFloor
from navigation_pr2.msg import ChangeFloorAction, ChangeFloorGoal

class Idling(smach.State):
    def __init__(self, client):
        smach.State.__init__(self, outcomes=['timeout', 'start navigation', 'start mapping', 'end', 'aborted', 'intro', 'list spots'])
        self.speak = client
        self.multiple_floor = rospy.get_param('~multiple_floor', True)
        rospy.loginfo('waiting for spot_map_server/change_floor...')
        rospy.wait_for_service('/spot_map_server/change_floor')
        self.eus_floor = rospy.ServiceProxy('/spot_map_server/change_floor', ChangeFloor)
        if self.multiple_floor:
            self.ac = actionlib.SimpleActionClient('map_manager/change_floor', ChangeFloorAction)
            rospy.loginfo('waiting for map_manager/change_floor...')
            self.ac.wait_for_server()

    def execute(self, userdata):
        if not wait_for_speech(timeout=120):
            return 'timeout'
        # unicode -> str
        speech_raw = rospy.get_param('~speech_raw').encode('utf-8')
        rospy.delete_param('~speech_raw')
        if re.search(r'.*案内.*$', speech_raw):
            return 'start navigation'
        if re.search(r'.*覚え.*$', speech_raw):
            self.speak.say('はい。よろしくお願いします')
            return 'start mapping'
        if re.search(r'.*終了.*$', speech_raw):
            return 'end'
        if re.search(r'.*こんにちは.*$', speech_raw):
            self.speak.say('こんにちは。私はPR2です。')
            return 'intro'
        if re.search(r'.*どこに行け.*$', speech_raw):
            return 'list spots'
        if re.search(r'^(.*)階.*到着.*$', speech_raw) is not None:
            floor_name = re.search(r'^(.*)階.*到着.*$', speech_raw).group(1)
            if not floor_name in floors:
                self.speak.parrot(speech_raw)
                return 'aborted'
            self.speak.say('{}階ですね。'.format(floor_name))
            floor_name = floors[floor_name]
            self.eus_floor(floor=floor_name)
            if self.multiple_floor:
                self.speak.say('地図を変えるので待って下さい')
                goal = ChangeFloorGoal()
                goal.command = 3
                goal.floor = floor_name
                self.ac.send_goal(goal)
                rospy.loginfo('waiting for change_floor result...')
                self.ac.wait_for_result()
                self.speak.say('お待たせしました')
            rospy.set_param('~floor', floor_name)
            return 'aborted'
        self.speak.parrot(speech_raw)
        return 'aborted'

class Initialize(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'],
                             output_keys=['riding_position', 'adjust_riding'])
        rospy.loginfo('waiting for move_base_node/DWAPlannerROS...')
        self.dr = False
        try:
            self.dr = dynamic_reconfigure.client.Client('/move_base_node/DWAPlannerROS', timeout=10.0)
        except:
            pass
        if not self.dr:
            rospy.loginfo('failed waiting for move_base_node/DWAPlannerROS...')
        # self.ver = rospy.get_param('~move_base_version', 'noetic')

    def execute(self, userdata):
        userdata.riding_position = {}
        userdata.adjust_riding = {}
        if self.dr is not False:
            self.dr.update_configuration({"acc_lim_x" : 2.0})
            self.dr.update_configuration({"acc_lim_y" : 2.0})
            self.dr.update_configuration({"acc_lim_theta" : 2.2})
        # if self.ver == 'noetic':
        #     self.dr.update_configuration({"max_vel_theta" : 1.3})
        # else:
        #     self.dr.update_configuration({"max_rot_vel" : 1.3})
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
            userdata.person_name = person_name # str
            self.speak.say('{}さんですね。よろしくお願いします。'.format(person_name))
            return 'succeeded'
        return 'aborted'

class Shutdown(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])
        rospy.loginfo('waiting for move_base_node/DWAPlannerROS...')
        self.dr = False
        try:
            self.dr = dynamic_reconfigure.client.Client('/move_base_node/DWAPlannerROS', timeout=10.0)
        except:
            pass
        if not self.dr:
            rospy.loginfo('failed waiting for move_base_node/DWAPlannerROS...')
    def execute(self, userdata):
        if self.dr:
            self.dr.update_configuration({"acc_lim_x" : 2.5})
            self.dr.update_configuration({"acc_lim_y" : 2.5})
            self.dr.update_configuration({"acc_lim_theta" : 5.0})
        return 'succeeded'

class ListSpots(smach.State):
    def __init__(self, client):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.speak = client
        rospy.loginfo('waiting for spot_map_server/list_spots...')
        rospy.wait_for_service('spot_map_server/list_spots')
        self.srv = rospy.ServiceProxy('spot_map_server/list_spots', ListSpotName)

    def execute(self, userdata):
        resp = self.srv()
        if resp.names == []:
            self.speak.say('どこにも行けません')
            return 'succeeded'
        self.speak.say('案内できる場所を読み上げます')
        floor = ''
        for n, f in zip(resp.names, resp.floors):
            if f != floor:
                self.speak.say('{}階の'.format(f))
                floor = f
            self.speak.say(n)
        self.speak.say('以上です')
        return 'succeeded'

