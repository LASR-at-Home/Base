#!/usr/bin/env python
import rospy
import tf2_ros
import tf2_geometry_msgs
import tf2_sensor_msgs

from geometry_msgs.msg import PoseStamped, PoseArray
from robocup_receptionist.srv import TfTransform, TfTransformResponse


from geometry_msgs.msg import PoseStamped, Vector3Stamped, PointStamped, WrenchStamped
import PyKDL
import rospy
import tf2_ros
import copy


def tf_transform(msg):
    tf_response = TfTransformResponse()
    if msg.pose_array.header.frame_id != '':
        transformation = get_transform(source_frame=msg.pose_array.header.frame_id, target_frame=msg.target_frame.data, stamp = msg.pose_array.header.stamp)
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
        transformation = get_transform(source_frame=msg.pointcloud.header.frame_id, target_frame=msg.target_frame.data, stamp = msg.pointcloud.header.stamp)
        if transformation:
            new_pointcloud = tf2_sensor_msgs.do_transform_cloud(msg.pointcloud, transformation)
            tf_response.target_pointcloud = new_pointcloud
        else:
            print('Error: No transformation')
    if msg.point.header.frame_id != '':
        transformation = get_transform(source_frame=msg.point.header.frame_id, target_frame=msg.target_frame.data,stamp = msg.point.header.stamp)
        if transformation:
            new_point = do_transform_point(msg.point, transformation)
            tf_response.target_point = new_point
        else:
            print('Error: No transformation')

   
    return tf_response

def get_transform(source_frame, target_frame, stamp):
    """
        Converts to target frame
        Returns the pose in the target frame
    """
    assert(source_frame and target_frame)
    # print('source_frame', source_frame)
    # print('target_frame', target_frame)
    try:
        transformation = tfBuffer.lookup_transform(target_frame, source_frame, stamp, rospy.Duration(0.1))
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
tf2_ros.TransformRegistration().add(PointStamped, do_transform_point)


def transform_to_kdl(t):
    return PyKDL.Frame(PyKDL.Rotation.Quaternion(t.transform.rotation.x, t.transform.rotation.y,
                                                 t.transform.rotation.z, t.transform.rotation.w),
                       PyKDL.Vector(t.transform.translation.x,
                                    t.transform.translation.y,
                                    t.transform.translation.z))

if __name__ == '__main__':
    
    rospy.init_node('tf_transform_node')
    s = rospy.Service('tf_transform', TfTransform, tf_transform)
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    print('Ready to transform!')
    rospy.spin()
