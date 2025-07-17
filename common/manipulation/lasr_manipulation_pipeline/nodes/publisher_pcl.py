#!/usr/bin/env python
import rospy
import tf2_ros
from sensor_msgs.msg import PointCloud2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import sensor_msgs.point_cloud2 as pc2  # For inspecting point cloud content
from tf_pcl import pcl_transform

publish_count = 0


def callback(cloud_msg):
    global publish_count
    rospy.loginfo_once("Received point cloud in frame: %s", cloud_msg.header.frame_id)

    try:
        # Corrected direction: target frame is cloud_msg.frame_id, source is intermediate_link
        if not tf_buffer.can_transform(
            cloud_msg.header.frame_id,
            "intermediate_link",
            rospy.Time(0),
            rospy.Duration(1.0),
        ):
            rospy.logwarn_throttle(
                5.0, "No TF from intermediate_link to %s", cloud_msg.header.frame_id
            )
            return

        # Lookup transform: from intermediate_link to cloud_msg.header.frame_id
        transform = tf_buffer.lookup_transform(
            cloud_msg.header.frame_id,  # target frame (cloud is defined in this frame)
            "intermediate_link",  # source frame (we want to bring cloud into this frame)
            rospy.Time(0),
            rospy.Duration(1.0),
        )

        rospy.loginfo_once(
            "Transform found from intermediate_link to %s", cloud_msg.header.frame_id
        )
        rospy.loginfo_once(
            "TF translation: x=%.3f, y=%.3f, z=%.3f",
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        )

        # Apply the transform to the cloud
        transformed_cloud = pcl_transform(cloud_msg, transform, "intermediate_link")
        # transformed_cloud = do_transform_cloud(cloud_msg, transform)
        transformed_cloud.header.frame_id = "intermediate_link"
        transformed_cloud.header.stamp = cloud_msg.header.stamp
        pub.publish(transformed_cloud)
        publish_count += 1

        # Print the first point in the transformed cloud (if available)
        points = pc2.read_points(
            transformed_cloud, skip_nans=True, field_names=("x", "y", "z")
        )
        first_point = next(points, None)
        if first_point:
            rospy.loginfo_throttle(
                5.0, "First transformed point: x=%.2f y=%.2f z=%.2f", *first_point
            )

        rospy.loginfo_throttle(
            10.0, "Published transformed cloud count: %d", publish_count
        )

    except Exception as e:
        rospy.logwarn("Transform failed: %s", str(e))


# Initialize the ROS node
rospy.init_node("cloud_transformer")

# Create TF buffer and listener
tf_buffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tf_buffer)

# Subscribe to the original point cloud
sub = rospy.Subscriber(
    "/xtion/depth_registered/points", PointCloud2, callback, queue_size=1
)

# Publisher for the transformed point cloud
pub = rospy.Publisher("/transformed_cloud", PointCloud2, queue_size=1)

# Spin to keep the node alive
rospy.spin()
