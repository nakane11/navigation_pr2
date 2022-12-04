#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import smach
import re
import actionlib
from actionlib_msgs.msg import GoalStatus
import math
from tf import TransformListener
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from pr2_mechanism_msgs.srv import SwitchController
from navigation_pr2.srv import Path, ChangeFloor
from navigation_pr2.utils import *

class Navigation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['goal reached', 'start mapping'])

    def execute(self, userdata):
        return 'goal reached'

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
        self.speak = client
        rospy.wait_for_service('/spot_map_server/find_path')
        self.call = rospy.ServiceProxy('/spot_map_server/find_path', Path)

    def execute(self, userdata):
        goal_name = userdata.goal_spot
        self.speak.say('経路を探します。')
        res = self.call(goal_name=goal_name)
        if res.result == 0:
            userdata.waypoints = res.waypoints
            floor = rospy.get_param('~floor')
            if res.goal_floor != floor:
                goal_floor = [k for k, v in floors.items() if v == res.goal_floor][0]
                self.speak.say('{}は{}階にあります。'.format(goal_name.encode('utf-8'), goal_floor))
            userdata.next_point = -1
            return 'ready to move'
        elif res.result == 1:
            self.speak.say('すみません。どこにあるかわかりません。')
        elif res.result == 2:
            goal_floor = [k for k, v in floors.items() if v == res.goal_floor][0]
            self.speak.say('すみません。{}は{}階にありますがエレベータの場所を知らないので案内できません。'.format(goal_name.encode('utf-8'), goal_floor))
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
            # self.py_floor(command=1, floor=floor_name)
            return 'aborted'
        return 'aborted'

class CheckIfGoalReached(smach.State):
    def __init__(self, client):
        smach.State.__init__(self, outcomes=['succeeded', 'unreached', 'preempted', 'aborted'],
                             input_keys=['waypoints', 'next_point', 'goal_spot'],
                             output_keys=['next_point'])
        self.speak = client

    def execute(self, userdata):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        goal_name = userdata.goal_spot
        next_point = userdata['next_point']
        waypoints = userdata['waypoints']
        if next_point > -1:
            if waypoints[next_point].name.decode('utf-8') == goal_name:
                self.speak.say('{}に到着しました。'.format(goal_name.encode('utf-8')))
                return 'succeeded'
        userdata.next_point = next_point + 1
        if len(waypoints) <= next_point + 1:
            return 'aborted'
        return 'unreached'

class ExecuteState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted', 'move'],
                             input_keys=['waypoints', 'next_point'])

    def execute(self, userdata):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        waypoints = userdata['waypoints']
        index = userdata['next_point']
        target_point = waypoints[index]
        print("index:{}\n name:{}\n type:{}".format(index, target_point.name, target_point.type))
        if target_point.type == 0:
            return 'move'
        return 'succeeded'

class SendMoveTo(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'],
                             input_keys=['next_point', 'waypoints'])
        self.distance_tolerance = rospy.get_param('~waypoint_distance_tolerance', 0.5)
        self.max_retry = rospy.get_param("~max_retry", 8)
        self.base_frame_id = rospy.get_param('~base_frame_id','base_footprint')
        self.goal_frame_id = rospy.get_param('~goal_frame_id','map')
        self.listener = tf.TransformListener()
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    def execute(self, userdata):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        waypoints = userdata['waypoints']
        index = userdata['next_point']
        waypoint = waypoints[index]

        goal_msg = MoveBaseGoal()
        goal_msg.target_pose.header.frame_id = self.goal_frame_id
        goal_msg.target_pose.pose.position = waypoint.pose.position
        goal_msg.target_pose.pose.orientation = waypoint.pose.orientation
        rospy.loginfo('Executing move_base goal to position (x,y): %s, %s' %
                      (waypoint.pose.position.x, waypoint.pose.position.y))
        self.client.send_goal(goal_msg)

        retry_count = 0
        distance = 10
        while(distance > self.distance_tolerance):
            now = rospy.Time.now()
            self.listener.waitForTransform(self.goal_frame_id, self.base_frame_id, now, rospy.Duration(4.0))
            trans,rot = self.listener.lookupTransform(self.goal_frame_id, self.base_frame_id, now)
            distance = math.sqrt(pow(waypoint.pose.position.x-trans[0],2)+pow(waypoint.pose.position.y-trans[1],2))
            if self.preempt_requested():
                self.client.cancel_all_goals()
                self.service_preempt()
                return 'preempted'
            state = self.client.get_state()
            if state == GoalStatus.ABORTED or state == GoalStatus.PREEMPTED:
                rospy.logwarn('Move_base failed because server received cancel request or goal was aborted')
                if retry_count < self.max_retry:
                    rospy.logwarn('Retry send goals')
                    self.client.send_goal(goal_msg)
                    retry_count += 1
                    continue
                rospy.logwarn("Finally Move_base failed because server received cancel request or goal was aborted")
                return 'aborted'
            rospy.loginfo("{}m to the next waypoint.".format(distance))
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
           

                              
