#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import psutil
import rosgraph
import rosnode
import subprocess
import time
try:
    from xmlrpc.client import ServerProxy
except ImportError:
    from xmlrpclib import ServerProxy

import rospy
import math
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped
from navigation_pr2.srv import *


class RobotService(object):
    def __init__(self):
        self.name = 'robot_service'
        self.master = rosgraph.Master(self.name)

    def launch_node(self, pkg, node, node_name, args=[], remap={}, timeout=15.0, polling=1.0, wait=True):
        rospy.loginfo("Start running {}".format(node_name))
        proc_app = subprocess.Popen(
            ['/opt/ros/{0}/bin/rosrun'.format(os.environ['ROS_DISTRO']), pkg, node]\
            + args + (['__name:={}'.format(node_name)] if node_name else [])\
            + ["{0}:={1}".format(key, value) for key, value in remap.items()],
            close_fds=True)

        def nodes_ready():
            try:
                rospy.loginfo("Waiting for {} ready ...".format(node_name))
                pid = ServerProxy(rosnode.get_api_uri(
                    self.master, node_name, skip_cache=True)).getPid(self.name)
                if pid[0]:
                    return True
            except Exception:
                pass
            return False

        if wait:
            if not self.wait_until(nodes_ready, timeout=timeout, polling=polling):
                rospy.loginfo("Failed running {}".format(node_name))
                return False
        rospy.loginfo("Succeeded")
        return proc_app

    def kill_process(self, proc):
        try:
            parent = psutil.Process(proc.pid)
        except psutil.NoSuchProcess:
            return
        # 子プロセスを再帰的に全て取得
        # children = parent.children(recursive=True)
        # 対象プロセスを落とす
        proc.terminate()
        proc.wait()
        # 残っている子プロセスを終了させる
        # for child in children:
        #     if child.is_running():
        #         child.kill()

    def term_node(self, proc):
        if proc is not None:
            self.kill_process(proc)

    def wait_until(self, func, timeout=None, polling=1.0, *args, **kwargs):
        if timeout is not None:
            end_time = time.time() + timeout
        else:
            end_time = time.time()
        while (time.time() < end_time or timeout is None):
            cond = func(*args, **kwargs)
            if cond:
                return True
            time.sleep(polling)
        return False


class MapManager(object):
    
    def __init__(self):
        self.rs = RobotService()
        self.procs = {}
        self.current_floor = None
        rospy.on_shutdown(self.handler)
        self.listener = tf.TransformListener()
        self.initialpose = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
        rospy.Service('~change_floor', ChangeFloor, self.floor_cb)

    def handler(self):
        try:
            for key, proc in self.procs:
                self.rs.term_node(proc)
        except:
            pass

    def floor_cb(self, req):
        if req.command == 0:
            self.start_make_map(req.floor)
        elif req.command == 1:
            self.change_make_map(req.floor)
        elif req.command == 2:
            self.stop_make_map()
        elif req.command == 3:
            self.change_floor(req.floor)
        return ChangeFloorResponse()

    def set_current_floor(self, floor):
        self.current_floor = floor
        rospy.set_param('/current_floor', floor)

    def start_make_map(self, floor):
        self.stop_map_server()
        self.stop_amcl()
        self.start_tf_publisher(floor)
        self.start_gmapping(floor)
        self.set_current_floor(floor)

    def change_make_map(self, floor):
        self.start_map_saver(self.current_floor)
        while not os.path.exists('/tmp/raw_maps/{}.yaml'.format(self.current_floor)):
            continue
        rospy.loginfo('Successfully saved {}.yaml!'.format(self.current_floor))
        trans, rot = self.get_robotpose()
        self.stop_gmapping()
        self.stop_tf_publisher()
        self.start_tf_publisher(floor)
        self.set_initialpose(trans, rot)
        self.start_gmapping(floor)
        self.set_current_floor(floor)

    def stop_make_map(self):
        trans, rot = self.get_robotpose()
        self.start_map_saver(self.current_floor)
        while not os.path.exists('/tmp/raw_maps/{}.yaml'.format(self.current_floor)):
            continue
        rospy.loginfo('Successfully saved {}.yaml!'.format(self.current_floor))
        self.stop_gmapping()
        self.start_map_server(self.current_floor)
        self.start_amcl()
        time.sleep(8)
        self.set_initialpose(trans, rot)
        trans, rot = self.get_robotpose()
        print(trans, rot)

    def change_floor(self, floor):
        trans, rot = self.get_robotpose()
        self.stop_map_server()
        self.start_map_server(floor)
        self.set_initialpose(trans, rot)
        self.set_current_floor(floor)

    # initialpose
    def get_robotpose(self):
        try:
            (trans,rot) = self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        return trans, rot

    def set_initialpose(self, trans, rot):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = trans[0]
        msg.pose.pose.position.y = trans[1]
        msg.pose.pose.position.z = trans[2]
        msg.pose.pose.orientation.x = rot[0]
        msg.pose.pose.orientation.y = rot[1]
        msg.pose.pose.orientation.z = rot[2]
        msg.pose.pose.orientation.w = rot[3]
        msg.pose.covariance[6 * 0 + 0] = 0.5 * 0.5
        msg.pose.covariance[6 * 1 + 1] = 0.5 * 0.5
        msg.pose.covariance[6 * 5 + 5] = math.pi/12.0 *  math.pi/12.0
        self.initialpose.publish(msg)

    # make /map of /map frame
    def start_gmapping(self, floor):
        package = 'gmapping'
        executable = 'slam_gmapping'
        name = 'slam_gmapping_{}'.format(floor)
        args=['_odom_frame:=odom_combined', '_map_frame:=/map',
              '_xmin:=-50.0', '_ymin:=-50.0', '_xmax:=50.0', '_ymax:=50.0']
        remap_args = {'scan':'base_scan'}
        gmapping = self.rs.launch_node(package, executable, name, args=args, remap=remap_args)
        self.procs['gmapping'] = gmapping

    def stop_gmapping(self):
        if 'gmapping' in self.procs:
            self.rs.term_node(self.procs['gmapping'])

    # save /map as 6f.yaml
    def start_map_saver(self, floor):
        package = 'map_server'
        executable = 'map_saver'
        name = 'map_saver_{}'.format(floor)
        args=['-f', '/tmp/raw_maps/{}'.format(floor)]
        saver = self.rs.launch_node(package, executable, name, args=args, wait=False)
        self.procs['saver'] = saver

    def stop_map_saver(self):
        if 'saver' in self.procs:
            self.rs.term_node(self.procs['saver'])

    # serve /map topic of /map frame from 6f.yaml
    def start_map_server(self, floor):
        print(floor)
        package = 'map_server'
        executable = 'map_server'
        name = 'map_server_{}'.format(floor)
        args=['/tmp/raw_maps/{}.yaml'.format(floor)]
        server = self.rs.launch_node(package, executable, name, args=args)
        self.procs['server'] = server

    def stop_map_server(self):
        if 'server' in self.procs:
            self.rs.term_node(self.procs['server'])

    # publish /world->/6f and /6f->/map tf
    def start_tf_publisher(self, floor):
        package = 'tf'
        executable = 'static_transform_publisher'

        name_from_world = "world_to_{}".format(floor)
        args_from_world= ["0.000", "0.000", "0.000", "0", "0", "0", "/world", "/{}".format(floor), "100"]
        tf_from_world = self.rs.launch_node(package, executable, name_from_world, args=args_from_world)
        self.procs['tf_from_world'] = tf_from_world
        
        name_to_map = "floor_{}_to_map".format(floor)
        args_to_map= ["0.000", "0.000", "0.000", "0", "0", "0", "/{}".format(floor), "/map", "100"]
        tf_to_map = self.rs.launch_node(package, executable, name_to_map, args=args_to_map)
        self.procs['tf_to_map'] = tf_to_map

    def stop_tf_publisher(self):
        if 'tf_from_world' in self.procs:
            self.rs.term_node(self.procs['tf_from_world'])
        if 'tf_to_map' in self.procs:
            self.rs.term_node(self.procs['tf_to_map'])

    def start_amcl(self):
        amcl = subprocess.Popen(
            ['/opt/ros/{0}/bin/roslaunch'.format(os.environ['ROS_DISTRO']),
             'jsk_pr2_startup',
             'amcl_node.xml'],
            close_fds=True)
        self.procs['amcl'] = amcl

    def stop_amcl(self):
        if 'amcl' in self.procs:
            self.rs.term_node(self.procs['amcl'])

if __name__ == '__main__':
    rospy.init_node('map_manager')
    MapManager()
    rospy.spin()



