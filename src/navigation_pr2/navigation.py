#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import smach
import re
from geometry_msgs.msg import PoseStamped
from pr2_mechanism_msgs.srv import SwitchController
from navigation_pr2.srv import Path, ChangeFloor
from navigation_pr2.utils import *

class Navigation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['goal reached', 'start mapping'])

    def execute(self, userdata):
        return 'goal reached'

class CheckIfNavigationAvailable(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['true', 'false'],
                             input_keys=['map_available'])

    def execute(self, userdata):
        if userdata.map_available:
            return 'true'
        else:
            return 'false'

class SetGoal(smach.State):
    def __init__(self, client):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'],
                             output_keys=['goal_spot'])
        self.speak = client

    def execute(self, userdata):
        self.speak.say('どちらへ案内しますか')
        if not wait_for_speech(timeout=30):
            return 'aborted'
        speech_raw = rospy.get_param('~speech_raw')
        rospy.delete_param('~speech_raw')
        userdata.goal_spot = speech_raw
        self.speak.say('{}ですね'.format(speech_raw.encode('utf-8')))
        return 'succeeded'

class GetWaypoints(smach.State):
    def __init__(self, client):
        smach.State.__init__(self, outcomes=['ready to move', 'no path found'],
                             input_keys=['goal_spot'],
                             output_keys=['waypoints', 'next_point'])
        self.poses = []
        self.speak = client
        self.waypoints_sub = rospy.Subscriber('/spot_map_server/poses', PoseStamped, self.waypoints_cb)
        rospy.wait_for_service('find_path')
        self.call = rospy.ServiceProxy('find_path', Path)

    def waypoints_cb(self, msg):
        self.poses.append(msg.pose)

    def execute(self, userdata):
        goal_name = userdata.goal_spot
        self.poses = []
        self.speak.say('経路を探します。')
        ret = self.call(goal_name=goal_name)
        if ret.result:
            userdata.waypoints = self.poses
            userdata.next_point = -1
            return 'ready to move'
        else:
            self.speak.say('すみません。案内できません。')
            return 'no path found'

class GetSpeechinMoving(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['preempted', 'end', 'request interrupt', 'start mapping', 'aborted'])
        rospy.wait_for_service('/spot_map_server/change_floor')
        # rospy.wait_for_service('/map_manager/change_floor')
        self.eus_floor = rospy.ServiceProxy('/spot_map_server/change_floor', ChangeFloor)
        self.py_floor = rospy.ServiceProxy('/map_manager/change_floor', ChangeFloor)

    def execute(self, userdata):
        if not wait_for_speech(timeout=5):
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            return 'aborted'
        speech_raw = rospy.get_param('~speech_raw')
        rospy.delete_param('~speech_raw')
        if re.search(r'^.*待って.*$', speech_raw) is not None:
            return 'request interrupt'
        elif re.search(r'^(.*)階.*到着.*$', speech_raw) is not None:
            floor_name = re.search(r'^(.*)階.*到着.*$', speech_raw).group(1)
            self.speak.say('{}階ですね'.format(floor_name))
            self.eus_floor(floor=floor_name)
            self.py_floor(command=1, floor=floor_name)
            return 'aborted'
        return 'aborted'

class CheckGoal(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'unreached', 'preempted', 'aborted'],
                             input_keys=['waypoints', 'next_point'],
                             output_keys=['next_point'])

    def execute(self, userdata):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        next_point = userdata['next_point']
        waypoints = userdata['waypoints']
        if next_point > -1:
            pass
            # if waypoints[next_point] is close to goal
            # return 'succeeded'
        userdata.next_point = next_point + 1
        if len(waypoints) <= next_point + 1:
            return 'aborted'
        return 'unreached'

class CheckElevator(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['use', 'not use', 'aborted', 'preempted'],
                             input_keys=['next_point'],
                             output_keys=['target_floor'])

    def execute(self, userdata):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        return 'use'

class MoveToElevator(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'],
                             input_keys=['target_floor'])

    def execute(self, userdata):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        return 'succeeded'

class SendMoveTo(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'],
                             input_keys=['next_point', 'waypoints'])
        self.pub = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=1);

    def execute(self, userdata):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        waypoints = userdata['waypoints']
        index = userdata['next_point']
        goal = waypoints[index]
        return 'succeeded'

class Interrupt(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['resume', 'aborted'])

    def execute(self, userdata):
        if not wait_for_speech(timeout=5):
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            return 'aborted'
        speech_raw = rospy.get_param('~speech_raw')
        rospy.delete_param('~speech_raw')
        if re.findall('再開', speech_raw):
            return 'resume'
        return 'aborted'

class FinishNavigation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])
        rospy.wait_for_service('/pr2_controller_manager/switch_controller')
        self.controller = rospy.ServiceProxy('/pr2_controller_manager/switch_controller', SwitchController)

    def execute(self, userdata):
        # 手繋ぎをやめる
        ret = self.controller(['l_arm_controller'], [], None)
        return 'succeeded'

class StartNavigation(smach.State):
    def __init__(self, client):
        smach.State.__init__(self, outcomes=['succeeded'])
        rospy.wait_for_service('/pr2_controller_manager/switch_controller')
        self.controller = rospy.ServiceProxy('/pr2_controller_manager/switch_controller', SwitchController)
        self.speak = client

    def execute(self, userdata):
        self.speak.say('手を繋いで下さい')
        # 手繋ぎ
        ret = self.controller([], ['l_arm_controller'], None)
        print(ret)
        self.speak.say('案内を開始します')
        return 'succeeded'

def con_moving_child_term_cb(outcome_map):
    if outcome_map['SEND_WAYPOINT'] == 'succeeded' or outcome_map['TALK_IN_MOVING'] == 'succeeded' or outcome_map['HAND_IMPACT'] == 'succeeded':
        return True
    if outcome_map['TALK_IN_MOVING'] == 'interrupt':
        return True
    if outcome_map['SEND_WAYPOINT'] == 'aborted':
        return True
    if outcome_map['TALK_IN_MOVING'] == 'start mapping':
        return True
    else:
        return False

def con_moving_out_cb(outcome_map):
    if outcome_map['SEND_WAYPOINT'] == 'succeeded':
        return 'reached'
    if outcome_map['TALK_IN_MOVING'] == 'succeeded':
        return 'succeeded'
    if outcome_map['TALK_IN_MOVING'] == 'interrupt':
        return 'interrupt'
    if outcome_map['SEND_WAYPOINT'] == 'aborted':
        return 'aborted'
    if outcome_map['TALK_IN_MOVING'] == 'start mapping':
        return 'start mapping'
    if outcome_map['HAND_IMPACT'] == 'succeeded':
        return 'ask'
           

                              
