#!/usr/bin/env python

import tf2_geometry_msgs
import tf2_sensor_msgs

from geometry_msgs.msg import PoseArray
from tf_module.srv import TfTransform, TfTransformResponse

from geometry_msgs.msg import PoseStamped, PointStamped, Pose
# import PyKDL
import rospy
import tf2_ros


def tf_transform(msg):
    tf_response = TfTransformResponse()
    if msg.pose_array.header.frame_id != '':
        transformation = get_transform(source_frame=msg.pose_array.header.frame_id, target_frame=msg.target_frame.data)
        if transformation:
            pose_array = PoseArray()
            pose_array.header.frame_id = msg.target_frame.data
            pose_array.header.stamp = rospy.get_rostime()

            for pose in msg.pose_array.poses:
                new_pose = tf2_geometry_msgs.do_transform_pose(PoseStamped(pose=pose), transformation).pose
                pose_array.poses.append(new_pose)

            tf_response.target_pose_array = pose_array
        else:
            print('Error: No transformation')
    if msg.pointcloud.header.frame_id != '':
        transformation = get_transform(source_frame=msg.pointcloud.header.frame_id, target_frame=msg.target_frame.data)
        if transformation:
            new_pointcloud = tf2_sensor_msgs.do_transform_cloud(msg.pointcloud, transformation)
            tf_response.target_pointcloud = new_pointcloud
        else:
            print('Error: No trasnformation')
    if msg.point.header.frame_id != '':
        transformation = get_transform(source_frame=msg.point.header.frame_id, target_frame=msg.target_frame.data)
        if transformation:
            new_point = do_transform_point(msg.point, transformation)
            tf_response.target_point = new_point
        else:
            print('Error: No trasnformation')

    p = None
    if msg.source_frame.data != '':
        transformation = get_transform(source_frame=msg.source_frame.data, target_frame=msg.target_frame.data)
        print(transformation, '  theirs')
        # return [t.x, t.y, t.z], [r.x, r.y, r.z, r.w]
        lt_trans, lt_rot = lookupTransform(target_frame=msg.target_frame.data,source_frame=msg.source_frame.data,time= rospy.Time(0))
        print(lt_trans, lt_rot, '   mine ')
        p = Pose()
        # return [t.x, t.y, t.z], [r.x, r.y, r.z, r.w]
        p.position.x = lt_trans[0]
        p.position.y = lt_trans[1]
        p.position.z = lt_trans[2]
        p.orientation.x = lt_rot[0]
        p.orientation.y = lt_rot[1]
        p.orientation.z = lt_rot[2]
        p.orientation.w = lt_rot[3]
    tf_response.translation_and_rotation = p

    return tf_response


def get_transform(source_frame, target_frame):
    """
        Converts to target frame
        Returns the pose in the target frame
    """
    assert (source_frame and target_frame)
    # print('source_frame', source_frame)
    # print('target_frame', target_frame)
    try:
        transformation = tfBuffer.lookup_transform(target_frame, source_frame, rospy.Time(0), rospy.Duration(0.1))
        return transformation
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        print(e)


def do_transform_point(point, transform):
    p = transform_to_kdl(transform) * PyKDL.Vector(point.point.x, point.point.y, point.point.z)
    res = PointStamped()
    res.point.x = p[0]
    res.point.y = p[1]
    res.point.z = p[2]
    res.header = transform.header
    return res

def strip_leading_slash(s):
    return s[1:] if s.startswith("/") else s

def lookupTransform(target_frame=None, source_frame=None, time=None):
    assert (source_frame and target_frame)
    try:
        msg = tfBuffer.lookup_transform(strip_leading_slash(target_frame), strip_leading_slash(source_frame), time)
        t = msg.transform.translation
        r = msg.transform.rotation
        return [t.x, t.y, t.z], [r.x, r.y, r.z, r.w]
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        print(e)
        return None


tf2_ros.TransformRegistration().add(PointStamped, do_transform_point)


def transform_to_kdl(t):
    return PyKDL.Frame(PyKDL.Rotation.Quaternion(t.transform.rotation.x, t.transform.rotation.y,
                                                 t.transform.rotation.z, t.transform.rotation.w),
                       PyKDL.Vector(t.transform.translation.x,
                                    t.transform.translation.y,
                                    t.transform.translation.z))

def tranform_pose(pose, target_frame, stamp):
    try:
        listener.waitForTransform(target_frame, "map", stamp, rospy.Duration(1.0))
        expected_door_pose_transformed = tfBuffer.transform(pose, target_frame)
        return expected_door_pose_transformed
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logwarn("Failed to transform expected door pose to the laser scan frame.")
        return False

if __name__ == '__main__':
    rospy.init_node('tf_transform_node')
    s = rospy.Service('tf_transform', TfTransform, tf_transform)
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    print('Ready to transform!')
    rospy.spin()
