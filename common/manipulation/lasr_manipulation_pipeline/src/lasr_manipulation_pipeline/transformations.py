from typing import List

import rospy
import numpy as np
import tf.transformations

import tf2_geometry_msgs
import tf2_ros

from geometry_msgs.msg import Pose, PoseStamped


def tf_poses(
    poses: List[Pose],
    source_frame: str,
    target_frame: str,
    buffer: tf2_ros.Buffer,
) -> List[Pose]:
    """
    Transforms a list of Pose objects from source_frame to target_frame.

    Args:
        poses: List of geometry_msgs.msg.Pose to transform.
        source_frame: The frame the poses are currently expressed in.
        target_frame: The frame to transform the poses into.

    Returns:
        List of transformed Pose objects in the target_frame.
    """
    transformed_poses = []
    try:
        # Wait for the transform to be available
        buffer.can_transform(
            target_frame, source_frame, rospy.Time(0), timeout=rospy.Duration(2.0)
        )
        transform = buffer.lookup_transform(
            target_frame, source_frame, rospy.Time(0), rospy.Duration(1.0)
        )

        for pose in poses:
            stamped_pose = PoseStamped()
            stamped_pose.header.frame_id = source_frame
            stamped_pose.header.stamp = rospy.Time(0)  # latest available
            stamped_pose.pose = pose

            transformed = tf2_geometry_msgs.do_transform_pose(stamped_pose, transform)
            transformed_poses.append(transformed.pose)

    except (
        tf2_ros.LookupException,
        tf2_ros.ConnectivityException,
        tf2_ros.ExtrapolationException,
    ) as e:
        rospy.logerr(f"Transform error from {source_frame} to {target_frame}: {e}")
        return []

    return transformed_poses


def offset_grasps(grasps: List[Pose], dx: float, dy: float, dz: float) -> List[Pose]:
    """
    Apply an offset in the local grasp frame to each grasp pose.
    dx, dy, dz are offsets in the local x, y, z directions (e.g., dz=-0.05 moves back along the gripper approach axis).
    """
    offset_grasps = []

    for pose in grasps:
        # Convert quaternion to rotation matrix
        quat = [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]
        rot = tf.transformations.quaternion_matrix(quat)

        # Local offset in grasp frame
        offset_local = np.array([dx, dy, dz, 1.0])  # homogeneous

        # Apply offset: get new position in world frame
        pos = np.array([pose.position.x, pose.position.y, pose.position.z])
        offset_world = rot @ offset_local
        new_pos = pos + offset_world[:3]

        # Create new pose
        new_pose = Pose()
        new_pose.position.x = new_pos[0]
        new_pose.position.y = new_pos[1]
        new_pose.position.z = new_pos[2]
        new_pose.orientation = pose.orientation  # keep orientation unchanged

        offset_grasps.append(new_pose)

    return offset_grasps


def transform_to_matrix(transform_msg):
    """
    Converts a geometry_msgs/Transform message into a 4x4 transformation matrix.

    Args:
        transform_msg (geometry_msgs.msg.Transform): The transform to convert.

    Returns:
        np.ndarray: A 4x4 homogeneous transformation matrix.
    """
    translation = transform_msg.translation
    rotation = transform_msg.rotation

    # Convert quaternion to rotation matrix
    quat = [rotation.x, rotation.y, rotation.z, rotation.w]
    rotation_matrix = tf.transformations.quaternion_matrix(quat)  # 4x4

    # Set translation
    rotation_matrix[0, 3] = translation.x
    rotation_matrix[1, 3] = translation.y
    rotation_matrix[2, 3] = translation.z

    return rotation_matrix


def get_lift_direction(pose: Pose) -> float:
    """
    Determine if lifting should be positive or negative along z in base_footprint
    based on the orientation of the gripper.

    Returns:
        +1.0 if gripper z-axis points down in base_footprint (i.e., lift = +z)
        -1.0 if gripper z-axis points up (i.e., lift = -z)
    """
    # Convert pose orientation to rotation matrix
    quat = [
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ]
    rot = tf.transformations.quaternion_matrix(quat)

    # Get the gripper's z-axis in its own frame
    z_axis = np.array([0, 0, 1, 0])  # homogeneous vector

    # Transform z_axis into base_footprint frame
    z_axis_world = rot @ z_axis
    z_dir = z_axis_world[:3]

    # The higher the z_dir[2], the more "up" it points in base_footprint
    if z_dir[2] < 0:
        return -1.0  # gripper is upside down — lift upward (+z)
    else:
        return +1.0  # gripper is upright — lift downward (-z)
