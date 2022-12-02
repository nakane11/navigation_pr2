#!/usr/bin/env python
# -*- coding: utf-8 -*-

import networkx as nx
import rospy
import tf
import actionlib
import math
from navigation_pr2.utils import *
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import Pose
from visualization_msgs.msg import MarkerArray, Marker
from navigation_pr2.msg import RecordSpotAction, RecordSpotResult
from navigation_pr2.srv import ChangeFloor, ChangeFloorResponse
from navigation_pr2.srv import Path, PathResponse


class SpotMapServer(object):
    def __init__(self):
        self.graph_dict = {}
        self.label = 1
        self.current_node = None
        self.auto_map_enabled = False
        self.active_graph = nx.MultiGraph()
        self.active_graph_name = 'initial'

        self.listener = tf.TransformListener()
        self.node_pub = rospy.Publisher('~nodes', MarkerArray, queue_size=1)
        self.edge_pub = rospy.Publisher('~edges', MarkerArray, queue_size=1)        
        self.start = rospy.Service('~start', Empty, self.start_auto_map)
        self.stop = rospy.Service('~stop', Empty, self.stop_auto_map)
        self.floor = rospy.Service('~change_floor', ChangeFloor, self.change_floor_cb)
        self.find_path = rospy.Service('~find_path', Path, self.find_path_cb)
        self.add = actionlib.SimpleActionServer('~add', RecordSpotAction, execute_cb=self.add_spot_cb)
        self.add.start()
        rospy.Timer(rospy.Duration(0.5), self.timer_cb)

    def start_auto_map(self, req):
        self.auto_map_enabled = True
        rospy.loginfo('auto map enabled')
        return EmptyResponse()

    def stop_auto_map(self, req):
        self.auto_map_enabled = False
        rospy.loginfo('auto map disabled')        
        return EmptyResponse()

    def timer_cb(self, event):
        if self.auto_map_enabled:
            curr_pose = self.get_robotpose()
            if not curr_pose:
                return
            # 前のノードがなかった場合に追加
            if not self.current_node:
                self.add_spot(curr_pose)
                return
            prev_pose = self.active_graph.nodes[self.current_node]['pose']
            angle = math.degrees(compute_angle_between_poses(curr_pose, prev_pose))
            diff = compute_difference_between_poses(curr_pose, prev_pose)
            # 前のノードとposeが十分違う場合に追加
            if ((40 < angle < 320) and diff > 0.3) or diff > 0.8:
                self.add_spot(curr_pose)

        self.publish_markers()

    def publish_markers(self, action=0):
        node_array_msg = MarkerArray()
        id = 1
        for i in list(self.active_graph.nodes):
            node = self.active_graph.nodes[i]
            if not 'name' in node:
                pin = make_pin_marker(i, node['pose'], id, rospy.Time.now(), radius=0.27, color=[0.5,0.5,0.5])
            else:
                pin = make_pin_marker(i, node['pose'], id, rospy.Time.now(), radius=0.27)
            node_array_msg.markers.extend(pin)
            id += 1
        id = 1
        edge_array_msg = MarkerArray()
        for edge in list(self.active_graph.edges):
            p1 = self.active_graph.nodes[edge[0]]['pose']
            p2 = self.active_graph.nodes[edge[1]]['pose']
            l = make_line_marker(p1, p2, id, rospy.Time.now(), color=(1,0,0))
            edge_array_msg.markers.extend([l])
            id += 1
        self.node_pub.publish(node_array_msg)
        self.edge_pub.publish(edge_array_msg)
    
    def add_spot_cb(self, goal):
        if goal.command == 0:
            pose = self.get_robotpose()
            if pose:
                self.add_spot(pose)
        elif goal.command == 1:
            pose = self.get_robotpose()
            if pose:
                self.add_spot(pose, goal.name)
        elif goal.command == 2:
            self.remove_spot(name)
        
        result = RecordSpotResult()
        self.add.set_succeeded(result)
        return

    def change_floor_cb(self, req):
        print(req.floor)
        self.change_graph(req.floor)
        self.publish_markers(action=2)
        resp = ChangeFloorResponse()
        return resp

    def find_path_cb(self, req):
        goal = req.goal_name
        goal_graph = None
        goal_floor = None
        resp = PathResponse(result=0)

        # ゴールがあるgraphを探索
        for name, graph in self.graph_dict.items():
            if goal in list(graph.nodes):
                goal_graph = graph
                goal_floor = name
                break
        if goal_graph is None:
            if goal in list(self.active_graph.nodes):
                goal_graph = self.active_graph
                goal_floor = self.active_graph_name
        if goal_graph is None:
            resp.result = 1
            return resp
        try:
            #同じ階(エレベータを使わない)
            if self.active_graph_name == goal_floor:
                path_list = nx.shortest_path(goal_graph, source=self.current_node, target=goal)
            #現在と違う階の場合
            else:
                path_list_target_floor = nx.shortest_path(goal_graph, source=self.current_node, target=elevator)
                path_list_source_floor = nx.shortest_path(goal_graph, source=self.current_node, target=elevator)
        except Excetion as e:
            rospy.loginfo(e)
            resp.result = 2
            return resp
        # for i in path_list_target_floor:
        #     pose = self.active_graph.nodes[i]['pose']



    def get_robotpose(self):
        try:
            (trans,rot) = self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
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
                        node_name = name[0]
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
        self.active_graph.nodes[node_name]['pose'] = pose
        if name:
            self.active_graph.nodes[node_name]['name'] = True
        # range, action, description
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
        rospy.loginfo('nodes: {}'.format(list(self.active_graph.nodes)))


if __name__ == '__main__':
    rospy.init_node('spot_map_server')
    s = SpotMapServer()
    rospy.spin()
