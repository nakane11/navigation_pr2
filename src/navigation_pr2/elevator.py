#!/usr/bin/env python
# -*- coding: utf-8 -*-

import actionlib
import smach
import re
import rospy
import tf
from navigation_pr2.utils import *
from virtual_force_drag.msg import SwitchAction, SwitchGoal
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose

class TeachRidingPosition(smach.State):
    def __init__(self, client):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'],
                             input_keys=['riding_position'],
                             output_keys=['riding_position'])
        self.speak = client
        self.listener = tf.TransformListener()

    def execute(self, userdata):
        self.speak.say('私をエレベータにのせてください')
        rospy.loginfo('waiting ...')
        if wait_for_speech(timeout=300):
            speech_raw = rospy.get_param('~speech_raw').encode('utf-8')
            rospy.delete_param('~speech_raw')
            if re.search(r'腕.*$', speech_raw) is not None:
                pos = self.get_robotpose()
                tmp = userdata.riding_position
                floor_name = rospy.get_param('~floor')
                tmp[floor_name] = pos
                userdata.riding_position = tmp
                self.speak.say('はい')
                return 'succeeded'
        self.speak.parrot(speech_raw)
        return 'aborted'

    def get_robotpose(self):
        try:
            (trans,rot) = self.listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
        except Exception as e:
            rospy.logerr(e)
            return False
        pose = Pose()
        pose.position.x = trans[0]
        pose.position.y = trans[1]
        pose.position.z = trans[2]
        pose.orientation.x = rot[0]
        pose.orientation.y = rot[1]
        pose.orientation.z = rot[2]
        pose.orientation.w = rot[3]
        return pose

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
            self.start_odom = self.ri.odom()
            self.start = True

        rospy.loginfo('waiting ...')
        if wait_for_speech(timeout=120):
            speech_raw = rospy.get_param('~speech_raw').encode('utf-8')
            rospy.delete_param('~speech_raw')
            pub_msg = Twist()
            if re.search(r'ここ.*$', speech_raw) is not None:
                diff = self.start_odom.difference_position(self.ri.odom())
                floor_name = rospy.get_param('~floor')
                tmp = userdata.adjust_riding
                tmp[floor_name] = diff
                userdata.adjust_riding = tmp
                self.speak.say('はい')
                self.start = False
                return 'succeeded'
            elif re.search(r'.*左.*$', speech_raw) is not None:
                pub_msg.linear.y = 0.6
                self.speak.say('左')
                for i in range(3):
                    self.pub.publish(pub_msg)
                    rospy.sleep(0.1)
            elif re.search(r'.*右.*$', speech_raw) is not None:
                pub_msg.linear.y = -0.6
                self.speak.say('右')
                for i in range(3):
                    self.pub.publish(pub_msg)
                    rospy.sleep(0.1)
            elif re.search(r'.*前.*$', speech_raw) is not None:
                pub_msg.linear.x = 0.6
                self.speak.say('前')
                for i in range(3):
                    self.pub.publish(pub_msg)
                    rospy.sleep(0.1)
            elif re.search(r'.*後.*$', speech_raw) is not None:
                pub_msg.linear.x = -0.6
                self.speak.say('後ろ')
                for i in range(3):
                    self.pub.publish(pub_msg)
                    rospy.sleep(0.1)
            else:
                self.speak.parrot(speech_raw)
        else:
            self.speak.say('乗り込み位置を教えて下さい')
        return 'aborted'
        
class WaitforNextFloor(smach.State):
    def __init__(self, client):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])
        self.speak = client
        
    def execute(self, userdata):
        # 到着をまつ
        rospy.loginfo('waiting ...')
        if wait_for_speech(timeout=300):
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
                    self.speak.say('お待たせしました。'.format(self.floor_name))
                rospy.set_param('~floor', floor_name)
                return 'succeeded'
            else:
                self.speak.parrot(speech_raw)
        return 'aborted'

class MovetoExit(smach.State):
    def __init__(self, ri):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'],
                             input_keys=['adjust_riding'])
        self.ac = actionlib.SimpleActionClient('/lead_pr2_action', SwitchAction)
        rospy.loginfo('waiting for lead_pr2_action...')
        self.ac.wait_for_server()
        self.ri = ri

    def execute(self, userdata):
        # 降車位置に移動
        floor_name = rospy.get_param('~floor')
        diff = userdata.adjust_riding[floor_name]
        self.ri.go_pos_unsafe(x=diff[0], y=diff[1])
        # 腕を出す
        goal = SwitchGoal(switch=True)
        self.ac.send_goal(goal)
        return 'succeeded'

class MoveInsideElevator(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'],        
                             input_keys=[])

    def execute(self, userdata):
        # エレベータの中に移動
        return

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
