#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
import smach
import re
import numpy as np
import dynamic_reconfigure.client
from move_base_msgs.msg import MoveBaseAction
from geometry_msgs.msg import Twist
from pr2_gripper_sensor_msgs.msg import PR2GripperEventDetectorAction, PR2GripperEventDetectorGoal
import pr2_gripper_sensor_msgs.msg
from navigation_pr2.utils import *

class WaitforHandImpact(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'detected', 'preempted', 'aborted'])
        self.ac = actionlib.SimpleActionClient('/l_gripper_sensor_controller/event_detector', PR2GripperEventDetectorAction)
        self.ac.wait_for_server()
        self.count = 0
        
    def execute(self, userdata):
        rospy.sleep(0.8)
        goal = PR2GripperEventDetectorGoal()
        goal.command.trigger_conditions = pr2_gripper_sensor_msgs.msg.PR2GripperEventDetectorCommand.ACC
        goal.command.acceleration_trigger_magnitude = 10.0
        self.ac.send_goal(goal)
        if self.ac.wait_for_result(timeout=rospy.Duration(8)):
            ret = self.ac.get_result()
            if ret.data.acceleration_event is True:
                self.count += 1
                if self.count > 1:
                    return 'succeeded'
                else:
                    return 'succeeded'
            elif self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            else:
                self.count = 0
                return 'aborted'
        elif self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        else:
            self.count = 0
            return 'aborted'

class AskWhat(smach.State):
    def __init__(self, client):
        smach.State.__init__(self, outcomes=['timeout', 'interrupt', 'preempted'])
        self.speak = client
        self.ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.pub = rospy.Publisher('/base_controller/command', Twist, queue_size=1)
        
    def execute(self, userdata):
        self.ac.cancel_all_goals()
        self.pub.publish(Twist())
        self.speak.say('どうしましたか')
        if not wait_for_speech(timeout=10):
            self.speak.say('何でもありません')
            return 'timeout'
        speech_raw = rospy.get_param('~speech_raw')
        rospy.delete_param('~speech_raw')
        if re.findall('待って', speech_raw):
            self.speak.say('わかりました')
            return 'interrupt'
        return 'preempted'

class ChangeSpeed(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])
        self.dr = dynamic_reconfigure.client.Client('/move_base_node/DWAPlannerROS', timeout=5.0)

    def execute(self, userdata):
        self.dr.update_configuration({"max_trans_vel" : 0.55})
        self.dr.update_configuration({"max_vel_x" : 2.5})
        self.dr.update_configuration({"acc_lim_x" : 2.5})
        self.dr.update_configuration({"acc_lim_theta" : 5.0})
        return 'succeeded'
