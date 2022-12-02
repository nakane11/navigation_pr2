#!/usr/bin/env python3

from visualization_msgs.msg import MarkerArray
import rospy
from jsk_topic_tools import ConnectionBasedTransport

import pykakasi

class RepublishMarkerArray(ConnectionBasedTransport):

    def __init__(self):
        super(RepublishMarkerArray, self).__init__()
        self.pub = self.advertise("~output", MarkerArray, queue_size=1)

        self.kks = pykakasi.kakasi()
        self.kks.setMode('J', 'a')
        self.kks.setMode('H', 'a')
        self.kks.setMode('K', 'a')
        self.kks.setMode('r', 'Kunrei')
        self.converter = self.kks.getConverter()
                    
    def subscribe(self):
        self.sub = rospy.Subscriber('~input', MarkerArray, self._cb)

    def unsubscribe(self):
        self.sub.unregister()

    def _cb(self, msg):
        marker_array = MarkerArray()
        markers = []
        for marker in msg.markers:
            if marker.type == 9:
                marker.text = self.converter.do(marker.text)
            markers.append(marker)
        marker_array.markers = markers
        self.pub.publish(marker_array)

if __name__ == '__main__':
    rospy.init_node('republish_marker_array')
    RepublishMarkerArray()
    rospy.spin()
