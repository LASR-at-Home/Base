#!/usr/bin/env python3.8

import rospy
from lasr_manipulation_3d_completion.srv import CompleteShape, CompleteShapeResponse
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import rospkg
import torch
from lasr_manipulation_3d_completion import complete_pointcloud, build_and_load_model
import open3d as o3d

def denoise_with_open3d(np_points):
    """
    Denoise a point cloud using Open3D:
    1. Voxel downsampling to reduce point count and smooth.
    2. Statistical outlier removal to eliminate isolated noise.
    (Optional) Could use radius-based outlier removal instead.
    """
    # Convert from NumPy array to Open3D PointCloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np_points)

    # 1. Apply voxel downsampling with a specified voxel size (meters)
    pcd = pcd.voxel_down_sample(voxel_size=0.02)

    # 2. Remove statistical outliers:
    #    - nb_neighbors: number of neighbors to analyze per point
    #    - std_ratio: standard deviation threshold for outlier removal
    cl, ind = pcd.remove_statistical_outlier(nb_neighbors=15, std_ratio=2.0)
    pcd = cl

    # (Optional) Use radius-based outlier removal instead:
    # cl2, ind2 = pcd.remove_radius_outlier(nb_points=16, radius=0.05)
    # pcd = cl2

    # Convert cleaned Open3D point cloud back to a NumPy array
    return np.asarray(pcd.points)



class CompletionService:
    def __init__(self):
        rospy.loginfo("Initializing shape completion service...")

        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        rospack = rospkg.RosPack()
        model_path = rospack.get_path('lasr_manipulation_3d_completion') + '/models/3dsgrasp_model.pth'

        self.model = build_and_load_model(
            ckpt_path=model_path,
            name='SGrasp',
            num_pred=6144,
            num_query=96,
            knn_layer=1,
            trans_dim=384,
            map_location=self.device
        )
        self.model.eval()

        rospy.Service('/shape_completion/complete', CompleteShape, self.handle_completion)

        rospy.Subscriber("/segmented_cloud", PointCloud2, self.segmented_cloud_callback)

        self.pcl_pub = rospy.Publisher("/completed_cloud_vis", PointCloud2, queue_size=1)

        rospy.loginfo("Shape completion service & topic listener ready.")

    def handle_completion(self, req):
        return self.process_pointcloud(req.input_cloud)

    def segmented_cloud_callback(self, msg):
        rospy.loginfo("Received /segmented_cloud message, running completion...")
        response = self.process_pointcloud(msg)
        self.pcl_pub.publish(response.completed_cloud)
        rospy.loginfo("Published /completed_cloud_vis")
        # rospy.sleep(1.0)

    def process_pointcloud(self, pcl_msg):
        partial_np = np.array([
            p[:3] for p in pc2.read_points(pcl_msg, field_names=["x", "y", "z"], skip_nans=True)
        ])

        if partial_np.shape[0] == 0:
            rospy.logwarn("Received empty point cloud. Returning original.")
            return CompleteShapeResponse(completed_cloud=pcl_msg, success=False)

        rospy.loginfo(f"Partial point cloud has {partial_np.shape[0]} points.")

        try:
            partial_np = denoise_with_open3d(partial_np)
            rospy.loginfo(f"Denoised partial cloud has {partial_np.shape[0]} points.")
        except Exception as e:
            rospy.logwarn(f"Open3D denoising failed, skipping: {e}")

        complete_np = complete_pointcloud(self.model, partial_np, device=self.device)
        complete_np = denoise_with_open3d(complete_np)

        rospy.loginfo(f"Completed point cloud has {complete_np.shape[0]} points.")

        return CompleteShapeResponse(
            completed_cloud=pc2.create_cloud_xyz32(pcl_msg.header, complete_np),
            success=True
        )


if __name__ == "__main__":
    rospy.init_node("shape_completion_server")
    CompletionService()
    rospy.spin()
