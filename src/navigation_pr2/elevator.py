#!/usr/bin/env python
# -*- coding: utf-8 -*-

import actionlib
import smach
import re
import rospy
from navigation_pr2.utils import *
from virtual_force_drag.msg import SwitchAction, SwitchGoal
from navigation_pr2.srv import ChangeFloor
from navigation_pr2.msg import ChangeFloorAction, ChangeFloorGoal
from geometry_msgs.msg import Twist

class TeachRidingPosition(smach.State):
    def __init__(self, client, listener):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'],
                             input_keys=['riding_position'],
                             output_keys=['riding_position'])
        self.speak = client
        self.listener = listener

    def execute(self, userdata):
        self.speak.say('私をエレベータにのせてください')
        rospy.loginfo('waiting ...')
        if wait_for_speech(timeout=300):
            speech_raw = rospy.get_param('~speech_raw').encode('utf-8')
            rospy.delete_param('~speech_raw')
            if re.search(r'腕.*$', speech_raw) is not None:
                pos = get_robotpose(self.listener)
                tmp = userdata.riding_position
                floor_name = rospy.get_param('~floor')
                tmp[floor_name] = pos
                userdata.riding_position = tmp
                self.speak.say('はい')
                return 'succeeded'
        self.speak.parrot(speech_raw)
        return 'aborted'

class MovetoInside(smach.State):
    def __init__(self, client, ri):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'],        
                             input_keys=['adjust_riding'],
                             output_keys=['adjust_riding'])
        self.pub = rospy.Publisher('/input_vel', Twist, queue_size=1)
        self.ac = actionlib.SimpleActionClient('/lead_pr2_action', SwitchAction)
        rospy.loginfo('waiting for lead_pr2_action...')
        self.ac.wait_for_server()
        self.start = False
        self.speak = client
        self.ri = ri

    def execute(self, userdata):
        if not self.start:
            # tuckarm
            goal = SwitchGoal(switch=False)
            self.ac.send_goal(goal)
            self.ac.wait_for_result()
            self.speak.say('乗り込み位置を教えて下さい')
            self.start_odom = self.ri.odom
            self.start = True

        rospy.loginfo('waiting ...')
        if wait_for_speech(timeout=120):
            speech_raw = rospy.get_param('~speech_raw').encode('utf-8')
            rospy.delete_param('~speech_raw')
            pub_msg = Twist()
            if re.search(r'ここ.*$', speech_raw) is not None:
                diff = self.ri.odom.difference_position(self.start_odom)
                print('diff:{}'.format(diff))
                floor_name = rospy.get_param('~floor')
                tmp = userdata.adjust_riding
                tmp[floor_name] = diff
                userdata.adjust_riding = tmp
                self.speak.say('はい')
                self.start = False
                return 'succeeded'
            elif re.search(r'.*左.*$', speech_raw) is not None:
                pub_msg.linear.y = 0.3
                self.speak.say('左')
                for i in range(6):
                    self.pub.publish(pub_msg)
                    rospy.sleep(0.1)
            elif re.search(r'.*右.*$', speech_raw) is not None:
                pub_msg.linear.y = -0.3
                self.speak.say('右')
                for i in range(6):
                    self.pub.publish(pub_msg)
                    rospy.sleep(0.1)
            elif re.search(r'.*前.*$', speech_raw) is not None:
                pub_msg.linear.x = 0.3
                self.speak.say('前')
                for i in range(6):
                    self.pub.publish(pub_msg)
                    rospy.sleep(0.1)
            elif re.search(r'.*後.*$', speech_raw) is not None:
                pub_msg.linear.x = -0.3
                self.speak.say('後ろ')
                for i in range(6):
                    self.pub.publish(pub_msg)
                    rospy.sleep(0.1)
            else:
                self.speak.parrot(speech_raw)
        else:
            self.speak.say('乗り込み位置を教えて下さい')
        return 'aborted'
        
class WaitforNextFloor(smach.State):
    def __init__(self, client):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'],
                             input_keys=['adjust_riding'],
                             output_keys=['adjust_riding'])
        self.multiple_floor = rospy.get_param('~multiple_floor')
        self.speak = client
        self.eus_floor = rospy.ServiceProxy('/spot_map_server/change_floor', ChangeFloor)
        rospy.loginfo('waiting for spot_map_server/change_floor...')
        rospy.wait_for_service('spot_map_server/change_floor')
        self.eus_floor = rospy.ServiceProxy('/spot_map_server/change_floor', ChangeFloor)
        if self.multiple_floor:
            self.ac = actionlib.SimpleActionClient('map_manager/change_floor', ChangeFloorAction)
            rospy.loginfo('waiting for map_manager/change_floor...')
            self.ac.wait_for_server()
        
    def execute(self, userdata):
        current_floor_name = rospy.get_param('~floor')
        tmp = userdata.adjust_riding
        # 到着をまつ
        rospy.loginfo('waiting ...')
        if wait_for_speech(timeout=300):
            speech_raw = rospy.get_param('~speech_raw').encode('utf-8')
            rospy.delete_param('~speech_raw')
            if re.search(r'^(.*)階.*到着.*$', speech_raw) is not None:
                self.floor_name = re.search(r'^(.*)階.*$', speech_raw).group(1)
                self.speak.say('{}階ですね。ちょっと待ってください'.format(self.floor_name))
                floor_name = floors[self.floor_name]
                tmp[floor_name] = userdata.adjust_riding[current_floor_name]
                userdata.adjust_riding = tmp
                self.eus_floor(floor=floor_name)
                if self.multiple_floor:
                    goal = ChangeFloorGoal()
                    goal.command = 1
                    goal.floor = floor_name
                    self.ac.send_goal(goal)
                    rospy.loginfo('waiting for change_floor result...')
                    self.ac.wait_for_result()
                    self.speak.say('お待たせしました。'.format(self.floor_name))
                rospy.set_param('~floor', floor_name)
                return 'succeeded'
            else:
                self.speak.parrot(speech_raw)
        return 'aborted'

class MovetoExit(smach.State):
    def __init__(self, client, ri):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'],
                             input_keys=['adjust_riding'])
        self.ac = actionlib.SimpleActionClient('/lead_pr2_action', SwitchAction)
        rospy.loginfo('waiting for lead_pr2_action...')
        self.ac.wait_for_server()
        self.ri = ri
        self.speak = client

    def execute(self, userdata):
        # 降車位置に移動
        floor_name = rospy.get_param('~floor')
        diff = userdata.adjust_riding[floor_name]
        self.ri.go_pos_unsafe(x=diff[0], y=diff[1])
        # 腕を出す
        goal = SwitchGoal(switch=True)
        self.ac.send_goal(goal)
        self.speak.say('エレベータからおろして下さい')
        return 'succeeded'

class TeachOutsideElevator(smach.State):
    def __init__(self, client):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])
        self.speak = client

    def execute(self, userdata):
        floor_name = rospy.get_param('~floor')
        # 次の階のエレベータの位置を教える
        if wait_for_speech(timeout=300):
            speech_raw = rospy.get_param('~speech_raw').encode('utf-8')
            rospy.delete_param('~speech_raw')
            if re.search(r'^.*エレベータ.*$', speech_raw) is not None:
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
                self.speak.say('はい。記録しました')
                return 'succeeded'
            self.speak.parrot(speech_raw)
        self.speak.say('{}階のエレベータの位置を教えて下さい'.format(floor_name))
        return 'aborted'

class HoldDoor(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])

    def execute(self, userdata):
        # ドアを押さえておく
        return

class a(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['timeout', 'name received', 'end', 'request navigation', 'aborted', 'cancelled'],
                             output_keys=['new_spot_name'])

    def execute(self, userdata):
        # 到着をまつ
        return

class MoveInsideElevator(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'],        
                             input_keys=[])

    def execute(self, userdata):
        # エレベータの外に移動
        return

# 中からエレベータ外 -> エレベータ外に移動してから手繋ぎをする
# 他の場所からエレベータ外 -> エレベータ外に移動してから手繋ぎをやめる->中に移動->
