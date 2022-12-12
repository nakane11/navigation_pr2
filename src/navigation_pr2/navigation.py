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
from robothand.msg import StartHoldingAction, StartHoldingGoal
from navigation_pr2.msg import MoveWristAction, MoveWristGoal
from std_msgs.msg import Float32MultiArray
import numpy as np

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
        smach.State.__init__(self, outcomes=['ready to move', 'no path found', 'multiple goals'],
                             input_keys=['goal_spot'],
                             output_keys=['waypoints', 'next_point', 'floor_candidates', 'path_candidates'])
        self.speak = client
        rospy.loginfo('waiting for spot_map_server/find_path...')
        rospy.wait_for_service('/spot_map_server/find_path')
        self.call = rospy.ServiceProxy('/spot_map_server/find_path', Path)

    def execute(self, userdata):
        goal_name = userdata.goal_spot
        self.speak.say('経路を探します。')
        res = self.call(goal_name=goal_name)
        res.result = np.frombuffer(res.result, dtype=np.uint8)
        print(res.result)
        res.waypoints_length = np.frombuffer(res.waypoints_length, dtype=np.uint8)

        # 一つも見つからなかった場合
        if np.all(res.result == [1]):
            self.speak.say('すみません。どこにあるかわかりません。')
            return 'no path found'

        else:
            path_known={}
            path_unknown=[]
            waypoints_array = res.waypoints
            for result, floor, length in zip(res.result, res.goal_floor, res.waypoints_length):
                if result == 0:
                    path_known[floor] = waypoints_array[0:length]
                    waypoints_array = waypoints_array[length:]
                elif result == 2:
                   path_unknown.append(floor)

            # 場所はわかるが経路が一つも見つからなかった場合
            if path_known == {} and path_unknown != []:
                floor_str = ""
                for i, floor in enumerate(path_unknown):
                    goal_floor = [k for k, v in floors.items() if v == floor][0]
                    floor_str += goal_floor
                    print(floor_str)
                    if i!=len(path_unknown)-1:
                        floor_str+="階と"
                self.speak.say('すみません。{}は{}階にありますが行き方を知らないので案内できません。'.format(goal_name.encode('utf-8'), floor_str))
                return 'no path found'

            current_floor = rospy.get_param('~floor')
            floor_list = list(path_known.keys())

            # pathが一つ見つかった場合
            if len(floor_list) == 1:
                target_floor = floor_list[0]
                userdata.waypoints = path_known[floor_list[0]]
                if target_floor != current_floor:
                    target_floor = [k for k, v in floors.items() if v == current_floor][0]
                    self.speak.say('{}は{}階にあります。'.format(goal_name.encode('utf-8'), target_floor))
                userdata.next_point = -1
                return 'ready to move'

            # pathが一つ以上ある場合
            print("before:{}".format(floor_list))
            floor_list = sorted(floor_list, key=lambda x: abs(int(x)-int(current_floor)))
            print("after:{}".format(floor_list))
            userdata.floor_candidates = floor_list
            userdata.path_candidates = path_known
            return 'multiple goals'
            
        # if res.result == 0:
        #     userdata.waypoints = res.waypoints
        #     floor = rospy.get_param('~floor')
        #     if res.goal_floor != floor:
        #         goal_floor = [k for k, v in floors.items() if v == res.goal_floor][0]
        #         self.speak.say('{}は{}階にあります。'.format(goal_name.encode('utf-8'), goal_floor))
        #     userdata.next_point = -1
        #     return 'ready to move'
        # elif res.result == 1:
        #     self.speak.say('すみません。どこにあるかわかりません。')
        # elif res.result == 2:
        #     goal_floor = [k for k, v in floors.items() if v == res.goal_floor][0]
        #     self.speak.say('すみません。{}は{}階にありますがエレベータの場所を知らないので案内できません。'.format(goal_name.encode('utf-8'), goal_floor))
        # return 'no path found'

class SuggestGoals(smach.State):
    def __init__(self, client):
        smach.State.__init__(self, outcomes=['ready to move', 'rejected', 'rejected all'],
                             input_keys=['goal_spot', 'floor_candidates', 'path_candidates'],
                             output_keys=['waypoints', 'next_point', 'floor_candidates', 'path_candidates'])
        self.speak = client

    def execute(self, userdata):
        goal_name = userdata.goal_spot
        floor_candidates = userdata.floor_candidates
        path_candidates = userdata.path_candidates

        goal_floor = [k for k, v in floors.items() if v == floor_candidates[0]][0]
        self.speak.say('{}階の{}でよろしいでしょうか？'.format(goal_floor, goal_name.encode('utf-8')))
        while True:
            rospy.loginfo('waiting for reply...')
            if wait_for_speech(timeout=20):
                speech_raw = rospy.get_param('~speech_raw').encode('utf-8')
                rospy.delete_param('~speech_raw')
                if re.search(r'.*はい.*$', speech_raw) is not None:
                    self.speak.say('かしこまりました。')
                    userdata.waypoints = path_candidates[floor_candidates[0]]
                    userdata.next_point = -1
                    return 'ready to move'
                elif re.search(r'.*いいえ.*$', speech_raw) is not None:
                    break
                else:
                    continue
            else:
                break
        if len(floor_candidates) <= 1:
            self.speak.say('すみません。他に場所を知りません。')
            return 'rejected all'
        else:
            userdata.floor_candidates = floor_candidates[1:]
            path_candidates.pop(floor_candidates[0])
            userdata.path_candidates = path_candidates
            self.speak.say('それでは')
            return 'rejected'

class GetSpeechinMoving(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['preempted', 'end', 'request interrupt', 'start mapping', 'aborted'])
        self.multiple_floor = rospy.get_param('~multiple_floor')
        rospy.loginfo('waiting for spot_map_server/change_floor...')
        rospy.wait_for_service('/spot_map_server/change_floor')
        self.eus_floor = rospy.ServiceProxy('/spot_map_server/change_floor', ChangeFloor)
        if self.multiple_floor:
            rospy.loginfo('waiting for map_manager/change_floor...')
            rospy.wait_for_service('/map_manager/change_floor')
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
            if self.multiple_floor:
                self.py_floor(command=1, floor=floor_name)
            rospy.set_param('~floor', floor_name)
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
        self.use_hand = rospy.get_param('~use_hand')
        if self.use_hand:
            self.hand_client = actionlib.SimpleActionClient('start_hand_holding', StartHoldingAction)
            rospy.loginfo('waiting for start_hand_holding...')
            self.hand_client.wait_for_server()
        rospy.loginfo('waiting for pr2_controller_manager/switch_controller...')
        rospy.wait_for_service('/pr2_controller_manager/switch_controller')
        self.controller = rospy.ServiceProxy('/pr2_controller_manager/switch_controller', SwitchController)

    def execute(self, userdata):
        # 手繋ぎをやめる
        ret = self.controller(['l_arm_controller'], [], None)
        if self.use_hand:
            goal = StartHoldingGoal(command=3)
            self.hand_client.send_goal(goal)
            self.hand_client.wait_for_result()
        return 'succeeded'

class StartNavigation(smach.State):
    def __init__(self, client):
        smach.State.__init__(self, outcomes=['succeeded', 'timeout'])
        self.use_hand = rospy.get_param('~use_hand')
        if self.use_hand:
            self.hand_client = actionlib.SimpleActionClient('start_hand_holding', StartHoldingAction)
            rospy.loginfo('waiting for start_hand_holding...')
            self.hand_client.wait_for_server()
        self.wrist_client = actionlib.SimpleActionClient('moving_wrist', MoveWristAction)
        rospy.loginfo('waiting for moving_wrist...')
        self.wrist_client.wait_for_server()
        rospy.loginfo('waiting for pr2_controller_manager/switch_controller...')
        rospy.wait_for_service('/pr2_controller_manager/switch_controller')
        self.controller = rospy.ServiceProxy('/pr2_controller_manager/switch_controller', SwitchController)
        self.speak = client

    def execute(self, userdata):
        self.speak.say('正面に立って下さい')
        goal = MoveWristGoal()
        self.wrist_client.send_goal(goal)
        if not self.wrist_client.wait_for_result(timeout=rospy.Duration(40)):
            self.wrist_client.cancel_all_goals()
            self.speak.say('中断しました')
            return 'timeout'
        self.speak.say('手を繋いで下さい')
        # 手繋ぎ
        if self.use_hand:
            goal = StartHoldingGoal(command=0)
            self.hand_client.send_goal(goal)
            if not self.hand_client.wait_for_result(timeout=rospy.Duration(40)):
                self.hand_client.cancel_all_goals()
                self.speak.say('中断しました')
                return 'timeout'
            print(self.hand_client.get_result())
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
    if outcome_map['HAND_IMPACT'] == 'succeeded':
        return 'ask'
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
           

                              
