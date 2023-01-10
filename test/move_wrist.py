#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
from navigation_pr2.msg import MoveWristAction, MoveWristGoal
from navigation_pr2 import SpeakClient

class MoveWrist():
    def __init__(self):
        rospy.init_node('move_wrist_test')
        self.wrist_client = actionlib.SimpleActionClient('moving_wrist', MoveWristAction)
        rospy.loginfo('waiting for moving_wrist...')
        self.wrist_client.wait_for_server()
        self.speak = SpeakClient()

    def execute(self):
        rospy.loginfo("sleep ...")
        rospy.sleep(10)
        rospy.loginfo("start")
        self.speak.say('手を繋ぎます')
        self.speak.say('正面に立って下さい')
        goal = MoveWristGoal()
        self.wrist_client.send_goal(goal)
        if not self.wrist_client.wait_for_result(timeout=rospy.Duration(40)):
            self.wrist_client.cancel_all_goals()
            self.speak.say('中断しました')
            return 'timeout'
        self.speak.say('手を繋いで下さい')
        return 'succeeded'

if __name__ == '__main__':
    mw = MoveWrist()
    mw.execute()
    rospy.spin()

    
