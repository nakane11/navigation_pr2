#!/usr/bin/env python

import networkx as nx
import rospy
import tf
import actionlib
import math
import numpy as np
import PyKDL
import tf_conversions.posemath as pm
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import Pose
from visualization_msgs.msg import MarkerArray
from navigation_pr2.msg import RecordSpotAction
from navigation_pr2.srv import ChangeFloor, ChangeFloorResponse
from navigation_pr2.srv import Path, PathResponse

def clockwise_angle_between_vectors(v1, v2, normal_vector):
    det = np.dot(normal_vector, np.cross(v1, v2))
    dot = np.dot(v1, v2)
    angle = np.arctan2(-det, -dot) + np.pi
    if np.isclose(angle, 2 * np.pi):
        angle = 0.0
    return angle

def compute_angle_between_poses(p1, p2):
        v1 = pm.fromMsg(p1).M * PyKDL.Vector(1,0,0)
        v1 = np.array([v1.x(), v1.y(), v1.z()])
        v2 = pm.fromMsg(p2).M * PyKDL.Vector(1,0,0)
        v2 = np.array([v2.x(), v2.y(), v2.z()])
        return clockwise_angle_between_vectors(v1, v2, [0,0,1])

def compute_difference_between_poses(p1, p2):
    diff = np.array([p1.position.x - p2.position.x,
                     p1.position.y - p2.position.y])
    return np.linalg.norm(diff)


class SpotMapServer(object):
    def __init__(self):
        self.graph_dict = {}
        self.label = 0
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
        rospy.Timer(rospy.Duration(0.5), self.auto_map_cb)

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
            if not self.current_node:
                self.add_spot(pose)
                return
            curr_pose = self.get_robotpose()
            if not curr_pose:
                return
            prev_pose = self.active_graph.nodes[self.current_node]['pose']
            angle = math.degrees(compute_angle_between_poses(curr_pose, prev_pose))
            diff = compute_difference_between_poses(curr_pose, prev_pose)
            if ((40 < angle < 320) and diff > 0.3) or diff > 0.8:
                self.add_spot(pose)

        self.publish_markers()

    def publish_markers(self):
        node_array_msg = MarkerArray()
        edge_array_msg = MarkerArray()
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
                self.add_spot(pose, (goal.name, goal.name_jp))
        elif goal.command == 2:
            self.remove_spot(goal.name_jp)
        
        result = RecordSpotResult()
        self.add.set_succeeded(result)
        return

    def change_floor_cb(self, req):
        self.change_graph(self, req.floor)
        resp = ChangeFloorResponse()
        return resp

    def find_path_cb(self, req):
        resp = PathResponse()
        return resp

    def get_robotpose(self):
        try:
            (trans,rot) = self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return
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
        if name is None:
            node_name = self.label
            label += 1
        else:
            node_name = name[0]

        if self.current_node and and name:
            if not 'name' in self.active_graph.nodes[self.current_node]:
                self.active_graph.nodes[self.current_node]['pose'] = pose
                self.active_graph.nodes[self.current_node]['name'] = name
        for n in list(self.active_graph.nodes):
            node = self.active_graph.nodes[n]
            if not node['name']
            
            
        self.active_graph.add_node(node_name)
        self.active_graph.nodes[node_name]['pose'] = pose
        self.active_graph.nodes[node_name]['name'] = name[1]
        # range, action, description
        return node_name

    def remove_spot(self, name=None, graph_name=None):
        if not name:
            self.active_graph.remove_node(self.current_node)
            return
        if graph_name:
            target_graph = self.graph_dict[graph_name]
            for n in list(target_graph.nodes):
                if self.taret_graph.nodes[n]['name'] == name:
                    self.target_graph.remove_node(n)
            self.graph_dict[graph_name] = target_graph
        else:
            target_graph = self.active_graph
            for n in list(target_graph.nodes):
                if self.taret_graph.nodes[n]['name'] == name:
                    self.target_graph.remove_node(n)
            self.active_graph = target_graph
        return
        
    def change_graph(self, name):
        self.graph_dict[self.active_graph_name] = self.active_graph
        if name in self.graph_dict:
            self.active_graph = self.graph_dict[name]
        else:
            self.active_graph = nx.MultiGraph()
        self.active_graph_name = name

    def publish_marker(self):
        return
    
if __name__ == '__main__':
    rospy.init_node('spot_map_server')
    s = SpotMapServer()
    rospy.spin()
