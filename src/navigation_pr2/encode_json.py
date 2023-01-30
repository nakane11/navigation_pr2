#!/usr/bin/env python

def encode_pose(pose):
    json = {}
    json['position'] = {'x': pose.position.x,
                        'y': pose.position.y,
                        'z': pose.position.z}
    json['orientation'] = {'x': pose.orientation.x,
                           'y': pose.orientation.y,
                           'z': pose.orientation.z,
                           'w': pose.orientation.w}
    return json
