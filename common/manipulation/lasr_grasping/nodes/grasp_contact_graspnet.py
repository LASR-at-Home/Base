#!/usr/bin/env python3
import rospy
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

import tf2_ros as tf
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose

import cv2_pcl
import cv2
import ultralytics
import numpy as np


sam = ultralytics.FastSAM("FastSAM-s.pt").to("cpu")

rospy.init_node("test_contact_graspnet_grasps")
clear_octomap = rospy.ServiceProxy("/clear_octomap", Empty)
tf_buffer = tf.Buffer(cache_time=rospy.Duration(10))
tf_listener = tf.TransformListener(tf_buffer)
moveit_commander.roscpp_initialize(sys.argv)

detect_grasps = rospy.ServiceProxy("/contact_graspnet/detect_grasps", DetectGrasps)

# Publisher for markers
marker_pub = rospy.Publisher("grasp_markers", Marker, queue_size=10)


def tf_pose(source_frame: str, target_frame: str, pose: Pose) -> Pose:
    try:
        transform = tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            rospy.Time(0),
            rospy.Duration(1.0),
        )
    except (
        tf.LookupException,
        tf.ConnectivityException,
        tf.ExtrapolationException,
    ) as e:
        raise rospy.ServiceException(str(e))

    pose_stamped = PoseStamped(pose=pose)
    pose_stamped.header.frame_id = source_frame
    return do_transform_pose(pose_stamped, transform)


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


def publish_marker(pose: Pose, frame_id: str, marker_id: int):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = "grasp_markers"
    marker.id = marker_id
    marker.type = Marker.ARROW  # you could also use Marker.AXES if you prefer
    marker.action = Marker.ADD
    marker.pose = pose
    marker.scale.x = 0.1  # Shaft length
    marker.scale.y = 0.02  # Shaft diameter
    marker.scale.z = 0.02  # Head diameter
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0
    marker.lifetime = rospy.Duration(30)  # Stays visible for 30 seconds
    marker_pub.publish(marker)


def create_collision_object_from_pcl(
    pcl_msg, planning_scene_interface, frame_id="base_footprint"
):
    # Convert PointCloud2 message to a list of (x, y, z)
    points = []
    for p in pc2.read_points(pcl_msg, field_names=("x", "y", "z"), skip_nans=True):
        points.append([p[0], p[1], p[2]])

    if not points:
        rospy.logwarn("No points in the PCL message!")
        return

    points = np.array(points)

    # Optional: remove outliers based on percentiles (e.g., filtering out the extreme points)
    # lower = np.percentile(points, 1, axis=0)
    # upper = np.percentile(points, 99, axis=0)

    # Filter points based on the percentile ranges
    # mask = np.all((points >= lower) & (points <= upper), axis=1)
    # points = points[mask]

    # Find the min and max of the points
    min_pt = points.min(axis=0)
    max_pt = points.max(axis=0)

    # Calculate the center of the bounding box and its size
    center = (min_pt + max_pt) / 2.0
    size = max_pt - min_pt

    p = PoseStamped()
    p.header.frame_id = frame_id
    # p.pose.position.x = obj.centroid.x
    # p.pose.position.y = obj.centroid.y
    # p.pose.position.z = obj.centroid.z
    p.pose.position.x = center[0]
    p.pose.position.y = center[1]
    p.pose.position.z = center[2]
    p.pose.orientation.w = 1
    planning_scene_interface.add_box("obj", p, size=size)

    # # Create the collision object (bounding box)
    # collision_object = CollisionObject()
    # collision_object.header.frame_id = frame_id
    # collision_object.id = "object"

    # # Define a SolidPrimitive (box)
    # box = SolidPrimitive()
    # box.type = SolidPrimitive.BOX
    # box.dimensions = [size[0], size[1], size[2]]

    # # Define the pose of the collision object
    # pose = Pose()
    # pose.position.x = center[0]
    # pose.position.y = center[1]
    # pose.position.z = center[2]
    # pose.orientation.w = 1.0  # No rotation, just the position

    # collision_object.primitives = [box]
    # collision_object.primitive_poses = [pose]
    # collision_object.operation = CollisionObject.ADD

    # Add the collision object to the planning scene
    # planning_scene_interface.apply_collision_object(collision_object)

    rospy.loginfo("Collision object added to the planning scene!")


# octomap_pub = rospy.Publisher("/throttle_filtering_points/filtered_points", PointCloud2)
# pcl_seg = None


# def update_octomap(_):
#     if pcl_seg is not None:
#         octomap_pub.publish(pcl_seg)


# rospy.Subscriber(
#     "/throttle_filtering_points/filtered_points", PointCloud2, update_octomap
# )
rospy.loginfo("Setting up MoveIt!")
arm_torso_group = moveit_commander.MoveGroupCommander("arm_torso")
arm_torso_group.set_planner_id("RRTConnectkConfigDefault")
arm_torso_group.allow_replanning(True)
arm_torso_group.allow_replanning(True)
arm_torso_group.allow_looking(True)
arm_torso_group.set_planning_time(15)
arm_torso_group.set_num_planning_attempts(10)
arm_torso_group.set_pose_reference_frame("base_footprint")
planning_scene = PlanningSceneInterface()
planning_scene.clear()
print(arm_torso_group.get_planning_frame())
print(arm_torso_group.get_end_effector_link())
rospy.loginfo("MoveIt! ready")
pcl_pub = rospy.Publisher("/segmented_cloud", PointCloud2)
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
create_collision_object_from_pcl(
    masked_cloud, planning_scene, masked_cloud.header.frame_id
)
# arm_torso_group.attach_object(
#     "obj",
#     touch_links=[
#         "gripper_left_finger_link",
#         "gripper_right_finger_link",
#         "gripper_link",
#     ],
# )
rospy.loginfo("Created colission object")
# Invert the mask: object == 0, background == 255
# inv_mask = cv2.bitwise_not(mask)

# # Find indices where background exists
# inv_indices = np.argwhere(inv_mask)

# # Read the original pointcloud (same as you did)
# pcl_xyz = np.array(
#     list(pc2.read_points(pcl, field_names=("x", "y", "z"), skip_nans=False)),
#     dtype=np.float32,
# )
# pcl_xyz = pcl_xyz.reshape((im.shape[0], im.shape[1], 3))
# # Select points outside the object
# background_points = pcl_xyz[inv_indices[:, 0], inv_indices[:, 1]]
# background_points = background_points[~np.isnan(background_points).any(axis=1)]
# background_points = background_points[~np.all(background_points == 0, axis=1)]

# # Create and publish pointcloud of background
# pcl_seg = background_cloud
# background_cloud = create_pointcloud2(background_points, pcl.header.frame_id)
# rospy.loginfo("Published background pointcloud without the segmented object")


rospy.loginfo("Got PCL, calling graspnet")
grasps = detect_grasps(pcl, masked_cloud).grasps
grasps = sorted(grasps, key=lambda g: g.score, reverse=True)
# for i, grasp in enumerate(gras)
arm_torso_group.set_pose_targets(
    [tf_pose(pcl.header.frame_id, "base_footprint", g.pose).pose for g in grasps],
    end_effector_link="gripper_grasping_frame",
)
res = arm_torso_group.go(wait=True)
print(res)
# print(arm_torso_group.go(wait=True))
# rospy.spin()
# /throttle_filtering_points/filtered_points
# for i, grasp in enumerate(grasps):
#     rospy.loginfo(f"Attempting grasp with score {grasp.score} {grasp.pose.position}")
#     pose = tf_pose(pcl.header.frame_id, "base_footprint", grasp.pose)

#     # Publish marker for each grasp
#     publish_marker(pose.pose, "base_footprint", i)

#     arm_torso_group.set_pose_target(pose.pose)
#     if arm_torso_group.go(wait=True):
#         break
#     # y = input("Continue?")
#     # if y[0] != "y":
#     #     break
