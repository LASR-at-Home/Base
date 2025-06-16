import rospy
from lasr_manipulation_msgs.srv import CompleteShape, CompleteShapeResponse
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import rospkg
import torch
from lasr_manipulation_3d_completion import complete_pointcloud, build_and_load_model


class CompletionService:
    def __init__(self):
        rospy.loginfo("Initializing shape completion service...")

        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        rospack = rospkg.RosPack()
        model_path = (
            rospack.get_path("lasr_manipulation_3d_completion")
            + "/models/3dsgrasp_model.pth"
        )

        self.model = build_and_load_model(
            ckpt_path=model_path,
            name="SGrasp",
            num_pred=6144,
            num_query=96,
            knn_layer=1,
            trans_dim=384,
            map_location=self.device,
        )
        self.model.eval()

        rospy.Service(
            "/shape_completion/complete", CompleteShape, self.handle_completion
        )

        # rospy.Subscriber("/segmented_cloud", PointCloud2, self.segmented_cloud_callback)

        self.pcl_pub = rospy.Publisher(
            "/completed_cloud_vis", PointCloud2, queue_size=1
        )

        rospy.loginfo("Shape completion service & topic listener ready.")

    def handle_completion(self, req):
        return self.process_pointcloud(req.input_cloud)

    def segmented_cloud_callback(self, msg):
        rospy.loginfo("Received /segmented_cloud message, running completion...")
        response = self.process_pointcloud(msg)
        self.pcl_pub.publish(response.completed_cloud)
        rospy.loginfo("Published /completed_cloud_vis")

    def process_pointcloud(self, pcl_msg):
        partial_np = np.array(
            [
                p[:3]
                for p in pc2.read_points(
                    pcl_msg, field_names=["x", "y", "z"], skip_nans=True
                )
            ]
        )

        if partial_np.shape[0] == 0:
            rospy.logwarn("Received empty point cloud. Returning original.")
            return CompleteShapeResponse(completed_cloud=pcl_msg, success=False)

        rospy.loginfo(f"Partial point cloud has {partial_np.shape[0]} points.")

        complete_np = complete_pointcloud(self.model, partial_np, device=self.device)

        rospy.loginfo(f"Completed point cloud has {complete_np.shape[0]} points.")

        return CompleteShapeResponse(
            completed_cloud=pc2.create_cloud_xyz32(pcl_msg.header, complete_np),
            success=True,
        )


if __name__ == "__main__":
    rospy.init_node("shape_completion_server")
    CompletionService()
    rospy.spin()
