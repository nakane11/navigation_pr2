#!/usr/bin/env python
import rospy
import numpy as np
import tf2_geometry_msgs
import tf2_ros
import PyKDL
from tf.transformations import unit_vector as normalize_vector
from jsk_recognition_msgs.msg import HandPose
from jsk_recognition_msgs.msg import HandPoseArray

class Silent():
    def __init__(self):
        self._tf_buffer = tf2_ros.Buffer()
        self._duration_timeout = rospy.get_param("~timeout", 3.0)
        self.base_frame_id = rospy.get_param("~base_frame_id", "base_link")
        self.sub = rospy.Subscriber('~input', HandPoseArray, self._convert)

    def _convert(self, msg):
        try:
            transform = self._tf_buffer.lookup_transform(
                self.base_frame_id,
                msg.header.frame_id,
                msg.header.stamp,
                timeout=rospy.Duration(self._duration_timeout))
            pykdl_transform_base_to_camera = tf2_geometry_msgs.transform_to_kdl(
                transform)
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn('{}'.format(e))
            return
        index_top_name = "index_mcp"
        index_bottom_name = "index_tip"

        for hand in msg.poses:
            if index_top_name in hand.finger_names and index_buttom_name in hand.finger_names:
                top_index = hand.finger_names.index(index_top_name)
                bottom_index = hand.finger_names.index(index_bottom_name)
                pose = hand.poses[top_index]
                top_pose = np.array([pose.position.x,
                                     pose.position.y,
                                     pose.position.z])
                pose = hand.poses[bottom_index]
                bottom_pose = np.array([pose.position.x,
                                        pose.position.y,
                                        pose.position.z])
                v_base = pykdl_transform_base_to_camera * PyKDL.Vector(*top_pose) \
                         - pykdl_transform_base_to_camera * PyKDL.Vector(*bottom_pose)
                v_base = np.array([v_base.x(), v_base.y(), v_base.z()])
                v_norm = np.linalg.norm(v_base)
                v_base = normalize_vector(v_base)

                rospy.loginfo("v_base:{}, {}, {}".format(*v_base))
                rospy.loginfo("norm:{}".format(v_norm))

                if v_base[2] > 0.7 and v_norm < 0.2:
                    rospy.set_param('/speak_node/volume', 0.0)
                    rospy.loginfo("set volume 0")
                    return
        rospy.sleep()

if __name__ == '__main__':
    rospy.init_node('silent')
    Silent()
    rospy.spin()
    
