import sys
import rospy

rospy.logwarn(sys.executable)
import ultralytics
import moveit_commander
import sys
from std_srvs.srv import Empty
from sensor_msgs.msg import PointCloud2
import sensor_msgs.msg
import std_msgs.msg
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Pose, PoseStamped
from visualization_msgs.msg import Marker
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject
from moveit_commander import PlanningSceneInterface
from ros_contact_graspnet.srv import DetectGrasps
from moveit_msgs.srv import GetPlanningScene, ApplyPlanningSceneRequest
from moveit_msgs.msg import PlanningScene, AllowedCollisionEntry
import tf2_ros as tf
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import actionlib
from play_motion_msgs.msg import PlayMotionGoal, PlayMotionAction

import cv2_pcl
import cv2
import numpy as np
from moveit.core.collision_detection import AllowedCollisionMatrix
import os
import rospkg

os.chdir(rospkg.RosPack().get_path("lasr_grasping"))

sam = ultralytics.FastSAM("FastSAM-s.pt").to("cpu")

rospy.init_node("pointcloud_segment_node")


def create_pointcloud2(points, frame_id="map"):
    """
    points: (N, 3) numpy array
    frame_id: TF frame
    returns: sensor_msgs/PointCloud2
    """
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id

    fields = [
        sensor_msgs.msg.PointField(
            name="x", offset=0, datatype=sensor_msgs.msg.PointField.FLOAT32, count=1
        ),
        sensor_msgs.msg.PointField(
            name="y", offset=4, datatype=sensor_msgs.msg.PointField.FLOAT32, count=1
        ),
        sensor_msgs.msg.PointField(
            name="z", offset=8, datatype=sensor_msgs.msg.PointField.FLOAT32, count=1
        ),
    ]

    # pack the points into a list of tuples
    points_list = [tuple(p) for p in points]

    pcl2_msg = pc2.create_cloud(header, fields, points_list)

    return pcl2_msg


# octomap_pub = rospy.Publisher("/throttle_filtering_points/filtered_points", PointCloud2)
# pcl_seg = None


# def update_octomap(_):
#     if pcl_seg is not None:
#         octomap_pub.publish(pcl_seg)


# rospy.Subscriber(
#     "/throttle_filtering_points/filtered_points", PointCloud2, update_octomap
# )

pcl_pub = rospy.Publisher("/segmented_cloud", PointCloud2, latch=True, queue_size=10)

pcl = rospy.wait_for_message("/xtion/depth_registered/points", PointCloud2)
rospy.loginfo("Got PCL")
im = cv2_pcl.pcl_to_cv2(pcl)


clicked_point = []


def mouse_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        print(f"Clicked at: ({x}, {y})")
        clicked_point.append((x, y))  # Save the point


# Load your image
image = im

# Display image
cv2.namedWindow("Image")
cv2.setMouseCallback("Image", mouse_callback)

while True:
    cv2.imshow("Image", image)
    key = cv2.waitKey(1) & 0xFF
    if key == 27:  # Press ESC to exit
        break
    if clicked_point:
        break

cv2.destroyAllWindows()

print(f"Selected point: {clicked_point[0]}")


rospy.loginfo("Got 2D Image, calling SAM")
result = sam(im, points=[list(clicked_point[0])], labels=[1])[0]
cv2.imshow("result", result.plot())
result = min(result, key=lambda r: r.boxes.xywh[0, 2] * r.boxes.xywh[0, 3])
cv2.waitKey(0)
cv2.imshow("result", result.plot())
cv2.waitKey(0)
cv2.destroyAllWindows()
xyseg = np.array(
    np.array(result.masks.xy).flatten().round().astype(int).tolist()
).reshape(-1, 2)
contours = xyseg.reshape(-1, 2)
mask = np.zeros(shape=im.shape[:2])
cv2.fillPoly(mask, pts=[contours], color=(255, 255, 255))
indices = np.argwhere(mask)
pcl_xyz = np.array(
    list(pc2.read_points(pcl, field_names=("x", "y", "z"), skip_nans=False)),
    dtype=np.float32,
)
pcl_xyz = pcl_xyz.reshape((im.shape[0], im.shape[1], 3))
masked_points = pcl_xyz[indices[:, 0], indices[:, 1]]
masked_points = masked_points[~np.isnan(masked_points).any(axis=1)]
masked_points = masked_points[~np.all(masked_points == 0, axis=1)]
masked_cloud = create_pointcloud2(masked_points, pcl.header.frame_id)
pcl_pub.publish(masked_cloud)
rospy.loginfo("Got segmentations")
