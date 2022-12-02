#!/usr/bin/env python

import rospy
import numpy as np
import math
import tf
import PyKDL
import tf_conversions.posemath as pm
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
from jsk_recognition_utils.color import labelcolormap
from geometry_msgs.msg import Point

N = 256
colors = labelcolormap(N=N) / 255.0

def wait_for_speech(timeout=0):
    start_time = rospy.Time.now()
    while not (rospy.has_param('~speech_raw')):
        if timeout > 0:
            if (rospy.Time.now() - start_time).to_sec() > timeout:
                return False
        continue
    return True

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

def make_cylinder(radius=0.1,
                  height=1.0,
                  pos=[0.0, 0.0, 0.0],
                  q_xyzw=[0.0, 0.0, 0.0, 1.0],
                  color=(0.0, 0.0, 0.0, 1.0),
                  lifetime=0.25,
                  id=0,
                  ns='',
                  frame_id='',
                  stamp=None):
    header = Header()
    header.frame_id = frame_id
    header.stamp = stamp

    mesh_marker = Marker(type=Marker.CYLINDER, header=header, id=id)
    mesh_marker.ns = ns
    mesh_marker.pose.position.x = pos[0]
    mesh_marker.pose.position.y = pos[1]
    mesh_marker.pose.position.z = pos[2]
    mesh_marker.pose.orientation.x = q_xyzw[0]
    mesh_marker.pose.orientation.y = q_xyzw[1]
    mesh_marker.pose.orientation.z = q_xyzw[2]
    mesh_marker.pose.orientation.w = q_xyzw[3]
    mesh_marker.scale.x = radius
    mesh_marker.scale.y = radius
    mesh_marker.scale.z = height
    mesh_marker.color.b, mesh_marker.color.g, mesh_marker.color.r, \
        mesh_marker.color.a = color
    mesh_marker.lifetime = rospy.Duration(lifetime)
    return mesh_marker

def make_cube(x=0.1, y=0.1, z=0.1,
              pos=[0.0, 0.0, 0.0],
              q_xyzw=[0.0, 0.0, 0.0, 1.0],
              color=(0.0, 0.0, 0.0, 1.0),
              lifetime=0.25,
              id=0,
              ns='',
              frame_id='',
              stamp=None):
    header = Header()
    header.frame_id = frame_id
    header.stamp = stamp

    mesh_marker = Marker(type=Marker.CUBE, header=header, id=id)
    mesh_marker.ns = ns
    mesh_marker.pose.position.x = pos[0]
    mesh_marker.pose.position.y = pos[1]
    mesh_marker.pose.position.z = pos[2]
    mesh_marker.pose.orientation.x = q_xyzw[0]
    mesh_marker.pose.orientation.y = q_xyzw[1]
    mesh_marker.pose.orientation.z = q_xyzw[2]
    mesh_marker.pose.orientation.w = q_xyzw[3]
    mesh_marker.scale.x = x
    mesh_marker.scale.y = y
    mesh_marker.scale.z = z
    mesh_marker.color.b, mesh_marker.color.g, mesh_marker.color.r, \
        mesh_marker.color.a = color
    mesh_marker.lifetime = rospy.Duration(lifetime)
    return mesh_marker

def make_text(string,
              scale=0.16,
              pos=[0.0, 0.0, 0.0],
              q_xyzw=[0.0, 0.0, 0.0, 1.0],
              color=(0.0, 0.0, 0.0, 1.0),
              lifetime=0.25,
              id=0,
              ns='',
              frame_id='',
              stamp=None):
    header = Header()
    header.frame_id = frame_id
    header.stamp = stamp

    mesh_marker = Marker(type=Marker.TEXT_VIEW_FACING, header=header, id=id)
    mesh_marker.ns = ns
    mesh_marker.pose.position.x = pos[0]
    mesh_marker.pose.position.y = pos[1]
    mesh_marker.pose.position.z = pos[2]
    mesh_marker.pose.orientation.x = q_xyzw[0]
    mesh_marker.pose.orientation.y = q_xyzw[1]
    mesh_marker.pose.orientation.z = q_xyzw[2]
    mesh_marker.pose.orientation.w = q_xyzw[3]
    mesh_marker.scale.x = scale
    mesh_marker.scale.y = scale
    mesh_marker.scale.z = scale
    mesh_marker.text = string
    mesh_marker.color.b, mesh_marker.color.g, mesh_marker.color.r, \
        mesh_marker.color.a = color
    mesh_marker.lifetime = rospy.Duration(lifetime)
    return mesh_marker

def make_line(pose1=[0.0, 0.0, 0.0],
              pose2=[0.0, 0.0, 0.0],
              scale=0.005,
              color=(0.0, 0.0, 0.0, 1.0),
              lifetime=25,
              id=0,
              ns='',
              frame_id='',
              stamp=None):
    header = Header()
    header.frame_id = frame_id
    header.stamp = stamp

    mesh_marker = Marker(type=Marker.LINE_LIST, header=header, id=id)
    mesh_marker.ns = ns
    p1 = Point(x=pose1[0], y=pose1[1], z=pose1[2])
    p2 = Point(x=pose2[0], y=pose2[1], z=pose2[2])
    mesh_marker.points.append(p1)
    mesh_marker.points.append(p2)
    mesh_marker.pose.orientation.w = 1
    mesh_marker.scale.x = scale
    mesh_marker.color.b, mesh_marker.color.g, mesh_marker.color.r, \
        mesh_marker.color.a = color
    mesh_marker.lifetime = rospy.Duration(lifetime)
    return mesh_marker

def make_pin_marker(name, pose, id, stamp, height = 1.8, radius= 0.1, color=None, alpha=0.7, lifetime=15, frame_id='/map'):
    if color is None:
        color = colors[id % N]
    cyl = make_cylinder(radius = radius, height = radius * 0.3,
                        pos = [pose.position.x, pose.position.y, height/3],
                        q_xyzw = list(tf.transformations.quaternion_from_euler(0, -math.pi/2, 0)),
                        color = tuple(color) + (alpha,),
                        lifetime = lifetime, id = id, ns='pin_head',frame_id = frame_id, stamp=stamp)
    cub = make_cube(x = radius/6, y = radius/6, z = height/2.5,
                    pos = [pose.position.x, pose.position.y, height/6],
                    color = tuple(color) + (alpha,),
                    lifetime = lifetime, id = id, ns='pin_body', frame_id = frame_id, stamp=stamp)
    txt = make_text(string=name,
                    pos = [pose.position.x, pose.position.y, height/6],
                    color = (1, 1, 1) + (1.0,),
                    lifetime = lifetime, id = id, ns='pin_label',frame_id = frame_id, stamp=stamp)

    return [cyl, cub, txt]

def make_line_marker(pose1, pose2, id, stamp, color=None, alpha=0.7, lifetime=15, frame_id='/map'):
    print(pose1)
    print(pose2)
    if color is None:
        color = colors[id % N]
    line = make_line([pose1.position.x, pose1.position.y, pose1.position.z],
                     [pose2.position.x, pose2.position.y, pose2.position.z],
                     id, color=tuple(color) + (alpha,), frame_id=frame_id, stamp=stamp)
    return line
