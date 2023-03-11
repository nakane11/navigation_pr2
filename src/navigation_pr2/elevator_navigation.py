#!/usr/bin/env python
# -*- coding: utf-8 -*-

import actionlib
import smach
import re
import rospy
import dynamic_reconfigure.client
from navigation_pr2.utils import *
from virtual_force_drag.msg import SwitchAction, SwitchGoal
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import Bool
from std_srvs.srv import Empty

class MovetoElevatorFront(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'],
                             input_keys=['next_point', 'waypoints'])
        self.max_retry = rospy.get_param("~max_retry", 3)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.goal_frame_id = rospy.get_param('~goal_frame_id','map')
        self.debug = rospy.get_param('~debug', False)

    def execute(self, userdata):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        waypoints = userdata['waypoints']
        index = userdata['next_point']
        waypoint = waypoints[index]
        print(waypoint)
        goal_msg = MoveBaseGoal()
        goal_msg.target_pose.header.frame_id = self.goal_frame_id
        goal_msg.target_pose.pose.position = waypoint.pose.position
        goal_msg.target_pose.pose.orientation = waypoint.pose.orientation
        rospy.loginfo('Executing move_base goal to position (x,y): %s, %s' %
                      (waypoint.pose.position.x, waypoint.pose.position.y))
        if self.debug:
            rospy.loginfo('skipped send_goal')
            return 'succeeded'
        else:
            self.client.send_goal(goal_msg)

        retry_count = 0
        while True:
            if self.preempt_requested():
                self.client.cancel_all_goals()
                self.service_preempt()
                return 'preempted'
            if retry_count > self.max_retry:
                return 'aborted'
            state = self.client.get_state()
            if state == GoalStatus.ABORTED or state == GoalStatus.PREEMPTED:
                rospy.logwarn('Move_base failed because server received cancel request or goal was aborted')
                if retry_count > self.max_retry:
                    self.client.cancel_all_goals()
                    return 'aborted'
                else:
                    rospy.logwarn('Retry send goals: {}'.format(retry_count))
                    self.client.send_goal(goal_msg)
                    retry_count += 1
                    continue
            elif state == GoalStatus.SUCCEEDED:
                return 'succeeded'
        return 'aborted'

class WaitforElevatorAvailable(smach.State):
    def __init__(self, client):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])
        self.ac = actionlib.SimpleActionClient('/lead_pr2_action', SwitchAction)
        rospy.loginfo('waiting for lead_pr2_action...')
        self.ac.wait_for_server()
        self.start = False
        self.speak = client
        self.mb = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    def execute(self, userdata):
        # tuckarm
        if not self.start:
            self.mb.cancel_all_goals()
            # tuckarm
            goal = SwitchGoal(switch=False)
            self.ac.send_goal(goal)
            self.ac.wait_for_result()
            self.start = True
        # 音声認識
        self.speak.say('エレベータが到着したら教えて下さい')
        if wait_for_speech(timeout=300):
            speech_raw = rospy.get_param('~speech_raw').encode('utf-8')
            rospy.delete_param('~speech_raw')
            if re.search(r'.*到着.*$', speech_raw) is not None:
                self.speak.say('私が乗り込むまでボタンを押しておいて下さい')
                self.start = False
                return 'succeeded'
            self.speak.parrot(speech_raw)
        return 'aborted'

class MovetoRidingPosition(smach.State):
    def __init__(self, ri, off):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'retry'],
                             input_keys=['riding_position'])
        self.dr = dynamic_reconfigure.client.Client('/move_base_node/global_costmap/static_layer', timeout=10.0)
        rospy.wait_for_service('/move_base/clear_costmaps')
        self.clear_costmaps = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        self.ri = ri
        self.off = off

    def execute(self, userdata):
        # if self.count == 0:
        self.dr.update_configuration({"enabled" : False})
        print(userdata.riding_position)
        floor_name = rospy.get_param('~floor')
        self.clear_costmaps()
        diff = userdata.riding_position[floor_name]
        print(diff)
        if self.off:
            self.ri.go_pos_unsafe(x=diff[0], y=diff[1], yaw=math.pi+diff[5], wait=True)
        else:
            self.ri.go_pos_unsafe(x=diff[0], y=diff[1], yaw=-diff[5], wait=True)
        self.dr.update_configuration({"enabled" : True})
        return 'succeeded'

class MovetoRidingPositionFailed(smach.State):
    def __init__(self, client):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'retry'])
        self.speak = client
        self.retry = 0

    def execute(self, userdata):
        if self.retry < 3:
            self.retry += 1
            return 'retry'
        self.speak.say('失敗しました。私をエレベータにのせてください')
        rospy.loginfo('waiting ...')
        if wait_for_speech(timeout=300):
            speech_raw = rospy.get_param('~speech_raw').encode('utf-8')
            rospy.delete_param('~speech_raw')
            if re.search(r'.*た.*$', speech_raw) is not None:
                self.speak.say('ありがとうございます')
                self.retry = 0
                return 'succeeded'
        self.speak.parrot(speech_raw)
        return 'aborted'

class MovetoInsidePosition(smach.State):
    def __init__(self, ri, off):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'],
                             input_keys=['adjust_riding'])
        self.ri = ri
        self.off = off

    def execute(self, userdata):
        floor_name = rospy.get_param('~floor')
        diff = userdata.adjust_riding[floor_name]
        # unsafeで移動
        if self.off:
            self.ri.go_pos_unsafe(x=diff[0], y=diff[1], wait=True)
        else:
            self.ri.go_pos_unsafe(x=-diff[0], y=-diff[1], wait=True)
        return 'succeeded'

class HoldDoor(smach.State):
    def __init__(self, model, ri, riding=True):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])
        self.r = model
        self.ri = ri
        self.riding = riding

    def execute(self, userdata):
        # 人が乗り込むまでドアを押さえておく
        self.r.rarm.angle_vector(np.deg2rad([19.3756, -6.43574, -36.3081, -26.6661, -196.366, -45.457, 230.06]))
        self.ri.angle_vector(self.r.angle_vector(), controller_type='rarm_controller')
        self.ri.wait_interpolation()
        self.r.head.angle_vector(np.deg2rad([-64.7933, 12.354]))
        self.ri.angle_vector(self.r.angle_vector(), controller_type='head_controller')
        self.ri.wait_interpolation()
        rospy.sleep(2)
        rospy.loginfo('wait for people...')
        start_time = rospy.Time.now()
        if self.riding:
            while True :
                ret = rospy.wait_for_message('/human_counter/output', Bool, timeout=2)
                print(ret)
                if ret.data or (rospy.Time.now() - start_time).to_sec() > 10:
                    break
        else:
            while True:
                ret = rospy.wait_for_message('/human_counter/output', Bool, timeout=2)
                print(ret)
                if not ret.data or (rospy.Time.now() - start_time).to_sec() > 10:
                    break
        # 乗り込んだらtuckarm
        rospy.sleep(5)
        self.r.rarm.angle_vector(np.deg2rad([-6.6127, 60.5828, -122.994, -74.8254, 56.2071, -5.72958, 10.8427]))
        self.ri.angle_vector(self.r.angle_vector(), controller_type='rarm_controller')
        self.ri.wait_interpolation()
        return 'succeeded'
