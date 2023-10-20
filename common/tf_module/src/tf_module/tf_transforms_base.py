#!/usr/bin/env python
import rospy
import tf2_ros
import tf2_geometry_msgs
import tf2_sensor_msgs

from geometry_msgs.msg import PoseStamped, PoseArray, Pose, TransformStamped
from tf_module.srv import TfTransform, TfTransformResponse, LatestTransform, LatestTransformResponse, ApplyTransform, \
    ApplyTransformResponse, BaseTransform, BaseTransformResponse

from geometry_msgs.msg import PoseStamped, Vector3Stamped, PointStamped, WrenchStamped
import PyKDL
import rospy
import tf2_ros
import copy


def tf_transform(msg):
    print(msg)
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
        transformation = get_transform(source_frame=msg.pointcloud.header.frame_id, target_frame=msg.target_frame.data, stamp=rospy.Time(0))
        if transformation:
            new_pointcloud = tf2_sensor_msgs.do_transform_cloud(msg.pointcloud, transformation)
            tf_response.target_pointcloud = new_pointcloud
        else:
            print('Error: No trasnformation')
    if msg.point.header.frame_id != '':
        transformation = get_transform(source_frame=msg.point.header.frame_id, target_frame=msg.target_frame.data, stamp=rospy.Time(0))
        if transformation:
            try:
                new_point = do_transform_point(msg.point, transformation)
                tf_response.target_point = new_point
            except Exception as e:
                print(e)
                print("---")
        else:
            print('Error: No trasnformation')

    p = None
    if msg.source_frame.data != '':
        transformation = get_transform(source_frame=msg.source_frame.data, target_frame=msg.target_frame.data, stamp=rospy.Time(0))
        print(transformation, '  theirs')
        # return [t.x, t.y, t.z], [r.x, r.y, r.z, r.w]
        lt_trans, lt_rot = lookupTransform(target_frame=msg.target_frame.data, source_frame=msg.source_frame.data,
                                           time=rospy.Time(0))
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


# KDL stuff
def do_transform_point(point, transform):
    p = transform_to_kdl(transform) * PyKDL.Vector(point.point.x, point.point.y, point.point.z)
    res = PointStamped()
    res.point.x = p[0]
    res.point.y = p[1]
    res.point.z = p[2]
    res.header = transform.header
    return res


tf2_ros.TransformRegistration().add(PointStamped, do_transform_point)


def transform_to_kdl(t):
    return PyKDL.Frame(PyKDL.Rotation.Quaternion(t.transform.rotation.x, t.transform.rotation.y,
                                                     t.transform.rotation.z, t.transform.rotation.w),
                           PyKDL.Vector(t.transform.translation.x,
                                        t.transform.translation.y,
                                        t.transform.translation.z))


# KDL stuff end

# base functions for transformations
def transform_point_from_given(point, from_frame="base_laser_link", to_frame="map"):
    try:
        tr = get_transform(from_frame, to_frame, rospy.Time(0))
        new_point = apply_transform(point, tr, is_input_ps=False)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logwarn("Failed to transform point.")
        rospy.logwarn(e)
        new_point = PointStamped()

    return new_point


def apply_transform(ps_, transform, is_input_ps=True, target="xtion_rgb_optical_frame"):
    """
        Apply transform of PointStamped or Point, with a given transform
    """
    # assert (ps_ and transform)
    if is_input_ps:
        tr_point = do_transform_point(ps_, transform)
    else:
        ps = PointStamped()
        ps.point = ps_
        try:
            tr_point = do_transform_point(ps, transform)
        except Exception as e:
            rospy.logwarn("Failed to transform point.")
            rospy.logwarn(e)
            print(transform)
            return PointStamped()

    return tr_point


def get_transform(source_frame, target_frame, stamp):
    """
        Converts to target frame
        Returns the pose in the target frame
    """
    # assert (source_frame and target_frame)
    try:
        transformation = tfBuffer.lookup_transform(target_frame, source_frame, stamp, rospy.Duration(0.1))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        print(e)
        transformation = TransformStamped()

    return transformation

# base functions for transformations

# callbacks
def transform_point_cb(msg):
    """
       Simply transform a point given the point and the frame
    """
    points = msg.points
    from_frame = msg.frame.data
    to_frame = msg.target_frame.data

    new_points = []
    for p in points:
        new_point = transform_point_from_given(p, from_frame, to_frame)
        new_points.append(new_point)
    return BaseTransformResponse(new_points)


def apply_transform_cb(msg):
    """
        The callback to apply transform
    """
    rospy.logwarn('Applying transform')
    new_p = []
    for p in msg.points:
        p_tf = apply_transform(p, msg.transform, is_input_ps=False)
        new_p.append(p_tf.point)
    return ApplyTransformResponse(new_p)


def get_latest_transform_cb(msg):
    """
        The callback to get the latest transform
    """
    to_frame = msg.target_frame
    from_frame = msg.from_frame
    t = TransformStamped()
    try:
        t = get_transform(source_frame=from_frame, target_frame=to_frame, stamp=rospy.Time(0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        print(e)

    return LatestTransformResponse(t)

# callbacks end

if __name__ == '__main__':
    rospy.init_node('tf_transform_node')
    s = rospy.Service('tf_transform', TfTransform, tf_transform)
    s2 = rospy.Service('latest_transform', LatestTransform, get_latest_transform_cb)
    s3 = rospy.Service('apply_transform', ApplyTransform, apply_transform_cb)
    s4 = rospy.Service('base_transform', BaseTransform, transform_point_cb)

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rospy.loginfo('Ready to transform!')
    rospy.spin()
