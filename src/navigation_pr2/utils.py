#!/usr/bin/env python

import rospy

def wait_for_speech(timeout=0):
    start_time = rospy.Time.now()
    while not rospy.has_param('~speech_roman'):
        if timeout > 0:
            if (rospy.Time.now() - start_time).to_sec() > timeout:
                return False
        continue
    return True
