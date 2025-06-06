#!/usr/bin/env python3.9

import os
import numpy as np

import rospy
import rospkg

import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Pose, PoseArray
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import (
    quaternion_from_matrix,
    quaternion_from_euler,
    quaternion_multiply,
)

from ros_contact_graspnet.srv import (
    DetectGrasps,
    DetectGraspsRequest,
    DetectGraspsResponse,
)
from ros_contact_graspnet.msg import Grasp

from contact_graspnet_pytorch.contact_grasp_estimator import GraspEstimator
from contact_graspnet_pytorch.checkpoints import CheckpointIO
from contact_graspnet_pytorch import config_utils

import math


class ContactGraspnetService:

    _grasp_offset = [0.0, 0.0, 0.0]

    def __init__(self):
        path = os.path.join(
            rospkg.RosPack().get_path("ros_contact_graspnet"),
            "checkpoints",
            "contact_graspnet",
        )
        self._estimator = GraspEstimator(config_utils.load_config(path))
        checkpoint = CheckpointIO(
            checkpoint_dir=os.path.join(path, "checkpoints"),
            model=self._estimator.model,
        )
        checkpoint.load("model.pt")
        rospy.Service(
            "contact_graspnet/detect_grasps", DetectGrasps, self.detect_grasps
        )
        self._marker_pub = rospy.Publisher(
            "/contact_graspnet/grasps", MarkerArray, queue_size=1
        )
        rospy.loginfo("ros_contact_graspnet service is ready!")

    def _pcl_msg_to_np(self, pcl_msg: PointCloud2) -> np.ndarray:
        return np.array(
            list(pc2.read_points(pcl_msg, field_names=("x", "y", "z"), skip_nans=True)),
            dtype=np.float32,
        )

    def _publish_grasps(self, grasps: PoseArray) -> None:
        marker_array = MarkerArray()

        for i, pose in enumerate(grasps.poses):
            marker = Marker()
            marker.header = grasps.header
            marker.id = i
            marker.type = Marker.ARROW
            marker.action = Marker.ADD

            marker.pose.position = pose.position
            marker.pose.orientation = pose.orientation

            marker.scale.x = 0.1
            marker.scale.y = 0.02
            marker.scale.z = 0.02

            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0

            marker_array.markers.append(marker)

        self._marker_pub.publish(marker_array)

    def detect_grasps(self, request: DetectGraspsRequest) -> DetectGraspsResponse:
        """
        request is a pointcloud
        convert pointcloud to numpy array,then call contact graspnet on it,
        return grasps in robots camera frame
        """
        response = DetectGraspsResponse()
        response.grasps.header = request.cloud.header
        points = self._pcl_msg_to_np(request.cloud)
        object_points = self._pcl_msg_to_np(request.object_cloud)
        pred_grasps_cam, scores, _, _ = self._estimator.predict_scene_grasps(
            points, {-1: object_points}, local_regions=True, filter_grasps=True
        )
        pred_grasps_cam = pred_grasps_cam[-1]
        scores = scores[-1]

        q_rot = quaternion_from_euler(
            -math.pi / 2.0, -math.pi / 2.0, 0.0, "sxyz"
        )  # rotation 1
        q_rot_180 = quaternion_from_euler(math.pi, 0.0, 0.0, "sxyz")  # rotation 2

        pred_grasps_cam[:, :3, 3] -= pred_grasps_cam[:, :3, 2] * 0.051

        for grasp, score in zip(pred_grasps_cam, scores):

            quat = quaternion_from_matrix(grasp)
            quat = quaternion_multiply(quat, q_rot)

            pose = Pose()
            pose.position.x = grasp[0, 3]
            pose.position.y = grasp[1, 3]
            pose.position.z = grasp[2, 3]

            pose.orientation.x = quat[0]
            pose.orientation.y = quat[1]
            pose.orientation.z = quat[2]
            pose.orientation.w = quat[3]

            pose_180 = Pose()
            pose_180.position = pose.position

            quat = quaternion_multiply(quat, q_rot_180)
            pose_180.orientation.x = quat[0]
            pose_180.orientation.y = quat[1]
            pose_180.orientation.z = quat[2]
            pose_180.orientation.w = quat[3]

            response.grasps.poses.append(pose)
            response.grasps.poses.append(pose_180)
            response.scores.append(score)
            response.scores.append(score)

        self._publish_grasps(response.grasps)

        return response


if __name__ == "__main__":
    rospy.init_node("contact_graspnet")
    service = ContactGraspnetService()
    rospy.spin()
