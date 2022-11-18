#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import smach
import re
import numpy as np
from geometry_msgs.msg import WrenchStamped
from navigation_pr2.utils import *

class GetVirtualForce(smach.State):
    def __init__(self, client):
        smach.State.__init__(self, outcomes=['preempted', 'wait'])
        self.pulled = False
        self.speak = client
        self.sub = rospy.Subscriber('/left_endeffector/wrench', WrenchStamped, self.callback)

    def callback(self, msg):
        v = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])
        self.pulled = np.linalg.norm(v) > 10
        
    def execute(self, userdata):
        rate = rospy.Rate(10)
        while True:
            if self.pulled:
                self.speak.say('どうしましたか')
                self.sub.unregister()
                return 'wait'
            elif self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            rate.sleep()
            
class AskWhat(smach.State):
    def __init__(self, client):
        smach.State.__init__(self, outcomes=['timeout', 'inturrupt', 'preempted'])
        self.speak = client
        
    def execute(self, userdata):
        if not wait_for_speech(timeout=10):
            self.speak.say('何でもありません')
            return 'timeout'
        speech_roman = rospy.get_param('~speech_roman')
        rospy.delete_param('~speech_roman')
        if re.findall('toma', speech_roman):
            return 'inturrupt'
        return 'preeempted'
