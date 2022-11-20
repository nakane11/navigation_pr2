#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
import smach
import re
import numpy as np
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
        goal = PR2GripperEventDetectorGoal()
        goal.command.trigger_conditions = pr2_gripper_sensor_msgs.msg.PR2GripperEventDetectorCommand.ACC
        goal.command.acceleration_trigger_magnitude = 30.0
        self.ac.send_goal(goal)
        if self.ac.wait_for_result(timeout=rospy.Duration(5)):
            ret = self.ac.get_result()
            self.count += 1
            if self.count > 2:
                return 'succeeded'
            else:
                return 'detected'
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
        
    def execute(self, userdata):
        self.speak.say('どうしましたか')
        if not wait_for_speech(timeout=10):
            self.speak.say('何でもありません')
            return 'timeout'
        speech_roman = rospy.get_param('~speech_roman')
        rospy.delete_param('~speech_roman')
        if re.findall('toma', speech_roman):
            return 'interrupt'
        return 'preeempted'
