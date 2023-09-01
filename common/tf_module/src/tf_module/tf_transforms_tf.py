#!/usr/bin/env python

import tf
from tf_module import transformations

from geometry_msgs.msg import PoseArray
from tf_module.srv import TfTransform, TfTransformResponse

from geometry_msgs.msg import PoseStamped, PointStamped, Pose
import PyKDL
import rospy


def tf_transform(msg):
    tf_response = TfTransformResponse()
    p = None
    if msg.source_frame.data != '':
        # listener.waitForTransform(msg.target_frame.data, msg.source_frame.data, rospy.Time(0), rospy.Duration(1))
        listener.waitForTransform(msg.target_frame.data, "map", rospy.Time(0), rospy.Duration(1))
        # listener.waitForTransform(msg.target_frame.data, msg.source_frame.data, rospy.Time(0), rospy.Duration(1))
        (lt_trans, lt_rot) = listener.lookupTransform(msg.target_frame.data, "map", rospy.Time(0))
        # (lt_trans, lt_rot) = listener.lookupTransform(msg.target_frame.data, msg.source_frame.data, rospy.Time(0))
        p = Pose()
        p.position.x = lt_trans[0]
        p.position.y = lt_trans[1]
        p.position.z = lt_trans[2]
        p.orientation.x = lt_rot[0]
        p.orientation.y = lt_rot[1]
        p.orientation.z = lt_rot[2]
        p.orientation.w = lt_rot[3]
    tf_response.translation_and_rotation = p

    return tf_response


if __name__ == '__main__':
    rospy.init_node('tf_transform_node')
    s = rospy.Service('tf_transform', TfTransform, tf_transform)
    listener = tf.TransformListener()

    print('Ready to transform!')
    rospy.spin()