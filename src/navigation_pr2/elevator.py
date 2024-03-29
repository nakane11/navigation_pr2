#!/usr/bin/env python
# -*- coding: utf-8 -*-

import actionlib
import smach
import re
import rospy
import dynamic_reconfigure.client
from navigation_pr2.utils import *
from navigation_pr2.encode_json import *
from navigation_pr2.decode_json import *
from virtual_force_drag.msg import SwitchAction, SwitchGoal
from navigation_pr2.srv import ChangeFloor
from navigation_pr2.msg import ChangeFloorAction, ChangeFloorGoal
from navigation_pr2.msg import RecordSpotAction, RecordSpotGoal
from geometry_msgs.msg import Twist
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


class TeachRidingPosition(smach.State):
    def __init__(self, client, listener, ri):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'],
                             input_keys=['riding_position'],
                             output_keys=['riding_position'])
        self.speak = client
        self.listener = listener
        self.start = False
        self.ri = ri

    def execute(self, userdata):
        # 初期位置を保存
        if not self.start:
            self.start_odom = self.ri.odom
            self.start = True

        self.speak.say('私をエレベータにのせてください')
        rospy.loginfo('waiting ...')
        if wait_for_speech(timeout=300):
            speech_raw = rospy.get_param('~speech_raw').encode('utf-8')
            rospy.delete_param('~speech_raw')
            if re.search(r'腕.*$', speech_raw) is not None:

                # 初期位置との差分
                difference_position = self.ri.odom.difference_position(self.start_odom) 
                difference_rotation = self.ri.odom.difference_rotation(self.start_odom)
                print('difference_position:{}'.format(difference_position))
                print('difference_rotation:{}'.format(difference_rotation))

                # userdataに保存
                tmp = userdata.riding_position
                floor_name = rospy.get_param('~floor')
                tmp[floor_name] = list(difference_position) + list(difference_rotation)
                userdata.riding_position = tmp

                self.start = False
                self.speak.say('はい')
                return 'succeeded'

        self.speak.parrot(speech_raw)
        return 'aborted'

class TeachInsidePosition(smach.State):
    def __init__(self, client, ri):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'],        
                             input_keys=['adjust_riding'],
                             output_keys=['adjust_riding'])
        self.debug = rospy.get_param('~debug', False)
        self.pub = rospy.Publisher('/input_vel', Twist, queue_size=1)

        self.ac = actionlib.SimpleActionClient('/lead_pr2_action', SwitchAction)
        rospy.loginfo('waiting for lead_pr2_action...')
        self.ac.wait_for_server()

        self.start = False
        self.speak = client
        self.ri = ri

    def execute(self, userdata):
        # 最初にtuckarm
        if not self.start:
            if self.debug:
                rospy.loginfo('skipped switchgoal')
            else:
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
                tmp[floor_name] = list(diff)
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
    def __init__(self, client, mapping=True):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'],
                             input_keys=['adjust_riding', 'riding_position'],
                             output_keys=['adjust_riding', 'riding_position'])
        self.multiple_floor = rospy.get_param('~multiple_floor', True)
        self.speak = client
        self.mapping=mapping
        rospy.loginfo('waiting for spot_map_server/change_floor...')
        rospy.wait_for_service('spot_map_server/change_floor')
        self.eus_floor = rospy.ServiceProxy('/spot_map_server/change_floor', ChangeFloor)
        if self.multiple_floor:
            self.ac = actionlib.SimpleActionClient('map_manager/change_floor', ChangeFloorAction)
            rospy.loginfo('waiting for map_manager/change_floor...')
            self.ac.wait_for_server()
        
    def execute(self, userdata):
        current_floor_name = rospy.get_param('~floor')
        tmp_a = userdata.adjust_riding
        tmp_b = userdata.riding_position

        # 到着をまつ
        rospy.loginfo('waiting ...')
        if wait_for_speech(timeout=300):
            speech_raw = rospy.get_param('~speech_raw').encode('utf-8')
            rospy.delete_param('~speech_raw')
            if re.search(r'^(.*)階.*到着.*$', speech_raw) is not None:
                self.floor_name = re.search(r'^(.*)階.*$', speech_raw).group(1)
                self.speak.say('{}階ですね。ちょっと待ってください'.format(self.floor_name))
                floor_name = floors[self.floor_name]
                # 新しい階にコピー
                tmp_a[floor_name] = userdata.adjust_riding[current_floor_name]
                tmp_b[floor_name] = userdata.riding_position[current_floor_name]
                userdata.adjust_riding = tmp_a
                userdata.riding_position = tmp_b
                # 地図を変更
                self.eus_floor(floor=floor_name)
                if self.multiple_floor:
                    goal = ChangeFloorGoal()
                    if self.mapping:
                        goal.command = 1
                    else:
                        goal.command = 3
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
        self.debug = rospy.get_param('~debug', False)
        self.ri = ri
        self.speak = client

    def execute(self, userdata):
        # 降車位置に移動
        floor_name = rospy.get_param('~floor')
        diff = userdata.adjust_riding[floor_name]
        if self.debug:
            rospy.loginfo('skipped go_pos_unsafe')
            return 'succeeded'
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
        self.ac = actionlib.SimpleActionClient('/spot_map_server/add', RecordSpotAction)
        rospy.loginfo('waiting for spot_map_server/add...')
        self.ac.wait_for_server()

    def execute(self, userdata):
        floor_name = rospy.get_param('~floor')
        # 次の階のエレベータの位置を教える
        if wait_for_speech(timeout=300):
            speech_raw = rospy.get_param('~speech_raw').encode('utf-8')
            rospy.delete_param('~speech_raw')
            if re.search(r'^.*エレベータ.*$', speech_raw) is not None:
                goal = RecordSpotGoal()
                goal.command = 3
                goal.name = 'エレベータ'
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
        # self.count = 0
        # self.max_retry = 6
        # self.mb = actionlib.SimpleActionClient('move_base', MoveBaseAction)
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
        # elevator_pos_json = userdata.riding_position[floor_name]
        # elevator_pos = decode_pose(elevator_pos_json)
        # goal_msg = MoveBaseGoal()
        # goal_msg.target_pose.header.frame_id = 'map'
        # goal_msg.target_pose.pose = elevator_pos
        # rospy.loginfo('Executing move_base goal to position (x,y): %s, %s' %
        #               (elevator_pos.position.x, elevator_pos.position.y))
        self.clear_costmaps()
        # self.mb.send_goal(goal_msg)
        # self.mb.wait_for_result(rospy.Duration(10))
        # state = self.mb.get_state()
        # if state == GoalStatus.ABORTED or state == GoalStatus.PREEMPTED:
        #     rospy.logwarn('Move_base failed because server received cancel request or goal was aborted')
        #     if self.count < self.max_retry:
        #         self.count += 1
        #         return 'retry'
        #     else:
        #         self.count = 0
        #         self.dr.update_configuration({"enabled" : True})
        #         return 'aborted'
        # self.count = 0
        diff = userdata.riding_position[floor_name]
        print(diff)
        if self.off:
            print('=============================')
            self.ri.go_pos_unsafe(x=-diff[0], y=-diff[1], yaw=-diff[5])
        else:
            print('|||||||||||||||||||||||||||||||')
            self.ri.go_pos_unsafe(x=diff[0], y=diff[1], yaw=diff[5])
        print(0)
        self.dr.update_configuration({"enabled" : True})
        print(1)
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
            self.ri.go_pos_unsafe(x=diff[0], y=diff[1])
        else:
            self.ri.go_pos_unsafe(x=-diff[0], y=-diff[1])
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
