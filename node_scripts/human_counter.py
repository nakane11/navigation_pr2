#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from jsk_recognition_msgs.msg import PeoplePoseArray

class HumanCounter(object):

    def __init__(self):
        self.count = [0,0,0,0,0,0,0,0,0,0]
        self.index = 0
        self.sub = rospy.Subscriber("/edgetpu_human_pose_estimator/output/poses", PeoplePoseArray, self.cb) 
        self.pub = rospy.Publisher("/human_counter/output", Bool, queue_size=1)
    
    def cb(self, msg):
        if msg.poses != []:
            self.count[self.index] = 1
        else:
            self.count[self.index] = 0
        self.index += 1
        if self.index == 10:
            ret = sum(self.count)/10.0
            if ret > 0.5:
                self.pub.publish(Bool(data=True))
            else:
                self.pub.publish(Bool(data=False))
            self.index = 0
            self.count = [0,0,0,0,0,0,0,0,0,0]

if __name__ == "__main__":
    rospy.init_node('human_counter')
    HumanCounter()
    rospy.spin()
