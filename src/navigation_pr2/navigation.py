#!/usr/bin/env python

import rospy
import smach

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

class GetWaypoints(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['ready to move', 'no path found'],
                             input_keys=['goal_spot'],
                             output_keys=['waypoints'])

    def execute(self, userdata):
        goal_name = userdata.goal_spot
        #send goal
        ret = True
        if ret:
            userdata.waypoints = ret
            return 'ready to move'
        else:
            return 'no path found'

class GetSpeechinMoving(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['preempted', 'end', 'request interrupt', 'start mapping', 'aborted'])

    def execute(self, userdata):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        rospy.sleep(20.0)
        return 'aborted'

class CheckGoal(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'unreached', 'preempted'],
                             output_keys=['next_point'])

    def execute(self, userdata):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        rospy.sleep(5)
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
        rospy.sleep(5)
        return 'use'

class MoveToElevator(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'],
                             input_keys=['target_floor'])

    def execute(self, userdata):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        rospy.sleep(5)
        return 'succeeded'

class SendMoveTo(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'],
                             input_keys=['next_point'])

    def execute(self, userdata):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        rospy.sleep(5)
        return 'succeeded'

class Interrupt(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['resume', 'aborted'])

    def execute(self, userdata):
        return 'resume'

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
    if outcome_map['SEND_WAYPOINT'] == 'succeeded' or outcome_map['TALK_IN_MOVING'] == 'succeeded':
        return 'succeeded'
    if outcome_map['TALK_IN_MOVING'] == 'interrupt':
        return 'interrupt'
    if outcome_map['SEND_WAYPOINT'] == 'aborted':
        return 'aborted'
    if outcome_map['TALK_IN_MOVING'] == 'start mapping':
        return 'start mapping'
    if outcome_map['HAND_IMPACT'] == 'succeeded':
        return 'ask'
           

                              
