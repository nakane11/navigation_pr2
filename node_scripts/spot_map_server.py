#!/usr/bin/env python
# -*- coding: utf-8 -*-

import struct
import threading
import networkx as nx
import rospy
import tf
import actionlib
import math
from threading import Lock
from navigation_pr2.utils import *
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import Pose
from visualization_msgs.msg import MarkerArray, Marker
from navigation_pr2.msg import RecordSpotAction, RecordSpotResult
from navigation_pr2.srv import ChangeFloor, ChangeFloorResponse
from navigation_pr2.srv import Path, PathResponse
from navigation_pr2.msg import Node
from navigation_pr2.srv import ListSpotName, ListSpotNameResponse

def convert_string_to_bytes(string):
    bytes = b''
    for i in string:
        bytes += struct.pack("B", ord(i))
    return bytes

class SpotMapServer(object):
    def __init__(self, lock):
        self.lock = lock
        self.graph_dict = {}
        self.label = 1
        self.current_node = None
        self.auto_map_enabled = False
        self.active_graph = nx.MultiGraph()
        self.active_graph_name = 'initial'

        self.pose_pub = rospy.Publisher('waypoints', PoseArray, queue_size=1)
        self.listener = tf.TransformListener()
        self.node_pub = rospy.Publisher('~nodes', MarkerArray, queue_size=1)
        self.edge_pub = rospy.Publisher('~edges', MarkerArray, queue_size=1)        
        self.start = rospy.Service('~start', Empty, self.start_auto_map)
        self.stop = rospy.Service('~stop', Empty, self.stop_auto_map)
        self.floor = rospy.Service('~change_floor', ChangeFloor, self.change_floor_cb)
        self.find_path = rospy.Service('~find_path', Path, self.find_path_cb)
        self.lsn = rospy.Service('~list_spots', ListSpotName, self.list_spot_name)
        self.add = actionlib.SimpleActionServer('~add', RecordSpotAction, execute_cb=self.add_spot_cb)
        self.add.start()

        self.thread = threading.Thread(target=self._loop)
        self.thread.daemon = True  # terminate when main thread exit
        self.thread.start()
        rospy.loginfo('initialized')

    def _loop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()
            if self.auto_map_enabled:
                if not self.listener.canTransform('/map', '/base_footprint', rospy.Time(0)):
                    rospy.loginfo('wait for transform')
                    continue
                curr_pose = self.get_robotpose()
                if not curr_pose:
                    rospy.loginfo('failed to get pose')
                    return
                with self.lock:
                    # 前のノードがなかった場合に追加
                    if not self.current_node:
                        self.add_spot(curr_pose)
                    else:
                        prev_pose = self.active_graph.nodes[self.current_node]['pose']
                        angle = math.degrees(compute_angle_between_poses(curr_pose, prev_pose))
                        diff = compute_difference_between_poses(curr_pose, prev_pose)
                        # 前のノードとposeが十分違う場合に追加
                        if ((40 < angle < 320) and diff > 0.3) or diff > 0.8:
                            self.add_spot(curr_pose)
            with self.lock:
                self.publish_markers()

    def start_auto_map(self, req):
        self.auto_map_enabled = True
        rospy.loginfo('auto map enabled')
        return EmptyResponse()

    def stop_auto_map(self, req):
        self.auto_map_enabled = False
        self.current_node = None
        rospy.loginfo('auto map disabled')        
        return EmptyResponse()

    def list_spot_name(self, req):
        name_list = []
        floor_list = []
        for floor_name, graph in self.graph_dict.items():
            for node_name in list(graph.nodes):
                if 'name' in graph.nodes[node_name] and graph.nodes[node_name]['name'] is True:
                    name_list.append(node_name)
                    floor_list.append(floor_name)
        for node_name in list(self.active_graph.nodes):
            if 'name' in self.active_graph.nodes[node_name] and self.active_graph.nodes[node_name]['name'] is True:
                name_list.append(node_name)
                floor_list.append(self.active_graph_name)

        resp = ListSpotNameResponse()
        resp.names = name_list
        resp.floors = floor_list
        return resp

    def publish_markers(self, action=0):
        node_array_msg = MarkerArray()
        id = 1
        for i in list(self.active_graph.nodes):
            node = self.active_graph.nodes[i]
            if not 'pose' in node:
                continue
            if not 'name' in node:
                pin = make_pin_marker(i, node['pose'], id, rospy.Time.now(), radius=0.27, color=[0.5,0.5,0.5])
            else:
                pin = make_pin_marker(i, node['pose'], id, rospy.Time.now(), radius=0.27)
            node_array_msg.markers.extend(pin)
            id += 1
        id = 1
        edge_array_msg = MarkerArray()
        for edge in list(self.active_graph.edges()):
            p1 = self.active_graph.nodes[edge[0]]['pose']
            p2 = self.active_graph.nodes[edge[1]]['pose']
            l = make_line_marker(p1, p2, id, rospy.Time.now(), color=(1,0,0))
            edge_array_msg.markers.extend([l])
            id += 1
        self.node_pub.publish(node_array_msg)
        self.edge_pub.publish(edge_array_msg)
    
    def add_spot_cb(self, goal):
        pose = self.get_robotpose()
        if not pose:
            rospy.loginfo('failed to get pose')
            result = RecordSpotResult(result=False)
            self.add.set_succeeded(result)
            return
        with self.lock:
            if goal.command == 0:
                self.add_spot(pose)
            elif goal.command == 1:
                print(goal.name)
                self.add_spot(pose, goal.name)
            elif goal.command == 2:
                self.remove_spot(goal.name)
            elif goal.command == 3:
                print(goal.name)
                self.add_spot(pose, goal.name)
                self.update_spot_info(goal.name, goal.node, goal.update_keys)
            elif goal.command == 4:
                print(goal.name)
                self.update_spot_info(goal.name, goal.node, goal.update_keys)
        result = RecordSpotResult(result=True)
        self.add.set_succeeded(result)
        return

    def change_floor_cb(self, req):
        with self.lock:
            self.change_graph(req.floor)
            self.publish_markers(action=2)
            resp = ChangeFloorResponse()
        return resp

    def find_path_cb(self, req):
        goal = req.goal_name
        print("goal:{}".format(goal))
        resp = PathResponse(result=0)
        result_array=[]
        goal_graph_dict = {}
        goal_floor_array=[]
        waypoints_array = []
        waypoints_length = []
        curr_pose = None
        while True:
            curr_pose = self.get_robotpose()
            if curr_pose is not False:
                break
            else:
                print('failed to get current position')
        start_node_candidates = list(self.active_graph.nodes)
        if len(start_node_candidates) > 0:
            start_node_candidates = sorted(start_node_candidates, key=lambda x: compute_difference_between_poses(curr_pose, self.active_graph.nodes[x]['pose']))
            start_node = start_node_candidates[0]

        # ゴールがあるgraphを探索
        for name, graph in self.graph_dict.items():
            goal_graph = None
            goal_floor = None
            print(type(goal))
            print(type(list(graph.nodes)[0]))
            for i in list(map(convert_string_to_bytes, list(graph.nodes))):
                print(i)
            if goal in map(convert_string_to_bytes, list(graph.nodes)):
                goal_graph_dict[name] = graph
                goal_floor = name
                goal_floor_array.append(goal_floor)
        for i in list(map(convert_string_to_bytes, list(self.active_graph.nodes))):
            print(i)

        if goal in map(convert_string_to_bytes, list(self.active_graph.nodes)):
            print("{} graph:{}".format(name,list(map(convert_string_to_bytes, list(self.active_graph.nodes)))))
            goal_graph_dict[self.active_graph_name] = self.active_graph
            goal_floor = self.active_graph_name
            goal_floor_array.append(goal_floor)

        if goal_floor_array is None:
            result_array = [1]

        for floor in goal_floor_array:
            goal_graph = goal_graph_dict[floor]
            print("floor{} goal_graph:{}".format(floor, list(map(convert_string_to_bytes, list(goal_graph.nodes)))))
            waypoints = []
            try:
                #同じ階
                if self.active_graph_name == floor:
                    path_list = nx.shortest_path(goal_graph, source=start_node, target=goal)
                    for i in path_list:
                        node = self.node_to_msg(i, self.active_graph.nodes[i])
                        waypoints.append(node)
                #現在と違う階の場合
                else:
                    for i in list(self.active_graph.nodes):
                        n = self.active_graph.nodes[i]
                        print("[source] name:{}, type:{}".format(i, n['type']))
                        if n['type'] == 1:
                            elevator_source = i
                            break
                    for i in list(goal_graph.nodes):
                        n = goal_graph.nodes[i]
                        print("[target] name:{}, type:{}".format(i, n['type']))
                        if n['type'] == 1:
                            elevator_target = i
                            break
                    if not (elevator_source and elevator_target):
                        result_array.append(2)
                        waypoints_length.append(0)
                        continue
                    if elevator_source and elevator_target:
                        path_list_source_floor = nx.shortest_path(self.active_graph, source=start_node, target=elevator_source)
                        path_list_target_floor = nx.shortest_path(goal_graph, source=elevator_target, target=goal)
                        for i in path_list_source_floor:
                            node = self.node_to_msg(i, self.active_graph.nodes[i])
                            waypoints.append(node)
                        for i in path_list_target_floor:
                            node = self.node_to_msg(i, goal_graph.nodes[i])
                            waypoints.append(node)
                result_array.append(0)
                waypoints = changeOrientation(waypoints)
                self.pose_pub.publish(publish_waypoints(waypoints))
                waypoints_array.extend(waypoints)
                waypoints_length.append(len(waypoints))

            except Exception as e:
                rospy.loginfo(e)
                result_array.append(2)
                waypoints_length.append(0)
        print("result:{}".format(result_array))
        print("length:{}".format(waypoints_length))
        resp.result = result_array
        resp.goal_floor = goal_floor_array
        resp.waypoints = waypoints_array
        resp.waypoints_length = waypoints_length
        return resp

    def node_to_msg(self, name, n):
        node_msg = Node()
        node_msg.name = name
        node_msg.floor = n['floor']
        node_msg.type = n['type']
        node_msg.pose = n['pose']
        node_msg.range = n['range'] if 'range' in n else 0.0
        node_msg.wait = n['wait'] if 'wait' in n else 0.0
        node_msg.description = n['description'] if 'description' in n else ''
        node_msg.read_out = n['read_out'] if 'read_out' in n else ''
        node_msg.keys = n['keys'] if 'keys' in n else []
        return node_msg

    def update_spot_info(self, name, node_info, update_keys):
        rospy.loginfo("update {}'s info: {}".format(name, update_keys))
        if not name in self.active_graph.nodes:
            rospy.loginfo("{} does not exist in current graph".format(name))
        for key in update_keys:
            if key == 'type':
                self.active_graph.nodes[name]['type'] = node_info.type
            elif key == 'range':
                self.active_graph.nodes[name]['range'] = node_info.range
            elif key == 'wait':
                self.active_graph.nodes[name]['wait'] = node_info.wait
            elif key == 'description':
                self.active_graph.nodes[name]['description'] = node_info.description
            elif key == 'read_out':
                self.active_graph.nodes[name]['read_out'] = node_info.read_out
            elif key == 'keys':
                self.active_graph.nodes[name]['keys'] = node_info.keys

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

    def add_spot(self, pose, name=None):
        # 一つ前のノードが自動記録されていて距離が近い場合に置き換え
        if self.current_node and name:
            prev_node = self.active_graph.nodes[self.current_node]
            if not 'name' in prev_node:
                prev_pose = prev_node['pose']
                diff = compute_difference_between_poses(pose, prev_pose)
                if diff < 0.25:
                    node_name = name
                    mapping = {self.current_node: node_name}
                    self.active_graph = nx.relabel_nodes(self.active_graph, mapping)
                    self.active_graph.nodes[node_name]['name'] = True
                    print(list(self.active_graph.nodes))
                    rospy.loginfo('Replace {} to {}'.format(self.current_node, node_name))
                    self.current_node = node_name
                    return
        for i in list(self.active_graph.nodes):
            node_i = self.active_graph.nodes[i]
            if not 'name' in node_i:
                pose_i = node_i['pose']
                diff = compute_difference_between_poses(pose, pose_i)
                if diff < 0.4:
                    if self.current_node and self.current_node != i:
                        self.active_graph.add_edge(self.current_node, i)
                        rospy.loginfo('Add edge {} to {}'.format(self.current_node, i))
                        self.current_node = i
                    if name :
                        node_name = name
                        mapping = {i: node_name}
                        self.active_graph = nx.relabel_nodes(self.active_graph, mapping)
                        self.active_graph.nodes[node_name]['name'] = True
                        rospy.loginfo('Replace {} to {}'.format(self.current_node, node_name))
                        self.current_node = node_name
                    return
        # 名前がない場合に数字を使用
        if name is None:
            node_name = str(self.label)
            self.label += 1
        else:
            node_name = name
        self.active_graph.add_node(node_name)
        rospy.loginfo('Add node {}'.format(node_name))
        self.active_graph.nodes[node_name]['type'] = 0
        self.active_graph.nodes[node_name]['pose'] = pose
        self.active_graph.nodes[node_name]['floor'] = self.active_graph_name
        if name:
            self.active_graph.nodes[node_name]['name'] = True
        if self.current_node:
            self.active_graph.add_edge(node_name, self.current_node)
            rospy.loginfo('Add edge {} to {}'.format(node_name, self.current_node))
        self.current_node = node_name
        return

    def remove_spot(self, name=None, graph_name=None):
        if not name:
            self.active_graph.remove_node(self.current_node)
            rospy.loginfo('Remove node {}'.format(self.current_node))
            self.current_node = None
            return
        if graph_name:
            target_graph = self.graph_dict[graph_name]
            for n in list(target_graph.nodes):
                if n == name:
                    target_graph.remove_node(n)
                    rospy.loginfo('Remove node {}'.format(n))
            self.graph_dict[graph_name] = target_graph
        else:
            target_graph = self.active_graph
            for n in list(target_graph.nodes):
                if n == name:
                    target_graph.remove_node(n)
                    rospy.loginfo('Remove node {}'.format(n))
            self.active_graph = target_graph
        return
        
    def change_graph(self, name):
        self.graph_dict[self.active_graph_name] = self.active_graph
        rospy.loginfo('Change graph from {} to {}'.format(self.active_graph_name, name))

        if name in self.graph_dict:
            self.active_graph = self.graph_dict[name]
        else:
            self.active_graph = nx.MultiGraph()
        self.active_graph_name = name
        self.current_node = None
        rospy.loginfo('nodes: {}'.format(list(self.active_graph.nodes)))


if __name__ == '__main__':
    lock = Lock()
    rospy.init_node('spot_map_server')
    s = SpotMapServer(lock)
    rospy.spin()
