#!/usr/bin/env python

from geometry_msgs.msg import Pose

def decode_pose(json):
    pose = Pose()
    pose.position.x = json['position']['x']
    pose.position.y = json['position']['y']
    pose.position.z = json['position']['z']
    pose.orientation.x = json['orientation']['x']
    pose.orientation.y = json['orientation']['y']
    pose.orientation.z = json['orientation']['z']
    pose.orientation.w = json['orientation']['w']
    return pose
