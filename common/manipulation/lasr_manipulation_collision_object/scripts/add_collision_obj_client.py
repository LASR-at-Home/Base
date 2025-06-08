#!/usr/bin/env python3
import rospy
from lasr_manipulation_collision_object.srv import GenerateCollisionObject
from sensor_msgs.msg import PointCloud2
from moveit_msgs.msg import CollisionObject


def test():
    rospy.init_node("test_collision_insert")
    rospy.loginfo("Node initialized")

    rospy.wait_for_service("completion_to_collision_object")
    service = rospy.ServiceProxy(
        "completion_to_collision_object", GenerateCollisionObject
    )

    # Wait for point cloud
    cloud = rospy.wait_for_message("/completed_cloud_vis", PointCloud2)

    # Call your service
    frame_id = "map"  # <- or base_footprint / odom, depending on your TF tree
    resp = service(cloud, "banana_obj", frame_id)

    # Publish collision object to planning scene
    pub = rospy.Publisher(
        "/collision_object", CollisionObject, queue_size=1, latch=True
    )
    rospy.sleep(1.0)  # allow time for connections
    pub.publish(resp.object)
    rospy.loginfo("Published collision object directly to /collision_object")

    rospy.sleep(2.0)  # allow RViz to catch up


if __name__ == "__main__":
    test()
