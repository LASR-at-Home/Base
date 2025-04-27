#!/usr/bin/env python3

import os
import numpy as np

import rospy
import rospkg

import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Pose
from tf.transformations import (
    quaternion_from_matrix,
    quaternion_from_euler,
    quaternion_multiply,
    euler_from_quaternion,
    euler_matrix,
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

    def __init__(self):
        rospy.Service(
            "contact_graspnet/detect_grasps", DetectGrasps, self.detect_grasps
        )
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

    def _pcl_msg_to_np(self, pcl_msg: PointCloud2) -> np.ndarray:
        return np.array(
            list(pc2.read_points(pcl_msg, field_names=("x", "y", "z"), skip_nans=True)),
            dtype=np.float32,
        )

    def detect_grasps(self, request: DetectGraspsRequest) -> DetectGraspsResponse:
        """
        request is a pointcloud
        convert pointcloud to numpy array,then call contact graspnet on it,
        return grasps in robots camera frame
        """
        response = DetectGraspsResponse()
        points = self._pcl_msg_to_np(request.cloud)
        object_points = self._pcl_msg_to_np(request.object_cloud)
        pred_grasps_cam, scores, _, _ = self._estimator.predict_scene_grasps(
            points, {-1: object_points}, local_regions=True, filter_grasps=True
        )
        pred_grasps_cam = pred_grasps_cam[-1]
        scores = scores[-1]
        q_rot = quaternion_from_euler(-math.pi / 2.0, -math.pi / 2.0, 0.0, "sxyz")

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

            grasp = Grasp(pose, score)
            response.grasps.append(grasp)

        return response


if __name__ == "__main__":
    rospy.init_node("contact_graspnet")
    service = ContactGraspnetService()
    rospy.spin()
