#!/usr/bin/env python
# -*- coding: utf-8 -*-

import actionlib
import smach
import rospy

from robothand.msg import StartHoldingAction, StartHoldingGoal
from pr2_mechanism_msgs.srv import SwitchController

class StopHandHolding(smach.State):
    def __init__(self, client):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])
        self.speak = client
        self.controller = rospy.ServiceProxy('/pr2_controller_manager/switch_controller', SwitchController)
        self.use_hand = rospy.get_param('~use_hand', False)
        self.debug = rospy.get_param('~debug', False)
        if self.use_hand:
            self.hand_client = actionlib.SimpleActionClient('start_hand_holding', StartHoldingAction)
            rospy.loginfo('waiting for start_hand_holding...')
            self.hand_client.wait_for_server()

    def execute(self, userdata):
        if self.use_hand:
            goal = StartHoldingGoal(command=3)
            self.hand_client.send_goal(goal)
            if not self.hand_client.wait_for_result(timeout=rospy.Duration(40)):
                self.hand_client.cancel_all_goals()
                self.speak.say('中断しました')
                return 'aborted'
        else:
            rospy.loginfo('skipped hand holding')
        if not self.debug:
            ret = self.controller(['l_arm_controller'], [], None)
        return 'succeeded'

class ResumeHandHolding(smach.State):
    def __init__(self, client):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])
        self.speak = client
        self.use_hand = rospy.get_param('~use_hand', False)
        self.controller = rospy.ServiceProxy('/pr2_controller_manager/switch_controller', SwitchController)
        self.debug = rospy.get_param('~debug', False)
        if self.use_hand:
            self.hand_client = actionlib.SimpleActionClient('start_hand_holding', StartHoldingAction)
            rospy.loginfo('waiting for start_hand_holding...')
            self.hand_client.wait_for_server()

    def execute(self, userdata):
        self.speak.say('手を繋いで下さい')
            # 手繋ぎ
        if self.use_hand:
            goal = StartHoldingGoal(command=0)
            self.hand_client.send_goal(goal)
            if not self.hand_client.wait_for_result(timeout=rospy.Duration(40)):
                self.hand_client.cancel_all_goals()
                self.speak.say('中断しました')
                return 'aborted'
            print(self.hand_client.get_result())
        else:
            rospy.loginfo('skipped hand holding')
        if not self.debug:
            ret = self.controller([], ['l_arm_controller'], None)
            print(ret)
        else:
            rospy.loginfo('skipped switch controller')
        self.speak.say('移動します')
        return 'succeeded'
