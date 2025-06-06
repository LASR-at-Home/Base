#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
from lasr_manipulation_3d_completion.srv import CompleteShape, CompleteShapeRequest
import sensor_msgs.point_cloud2 as pc2
# import sys
# rospy.init_node("test_3d_completion_client")
# rospy.logwarn(sys.executable)

def print_cloud_info(cloud_msg, name="PointCloud"):
    try:
        points = list(pc2.read_points(cloud_msg, skip_nans=True))
        rospy.loginfo(f"{name} has {len(points)} points.")
    except Exception as e:
        rospy.logwarn(f"Could not read points from {name}: {e}")

if __name__ == "__main__":
    rospy.init_node("shape_completion_client")

    rospy.loginfo("Waiting for partial point cloud...")
    cloud_msg = rospy.wait_for_message("/partial_cloud_vis", PointCloud2)
    rospy.wait_for_service("/shape_completion/complete")
    complete_srv = rospy.ServiceProxy("/shape_completion/complete", CompleteShape)

    req = CompleteShapeRequest()
    req.input_cloud = cloud_msg

    try:
        res = complete_srv(req)
        if res.success:
            print_cloud_info(res.completed_cloud, "Completed cloud")
            pub2 = rospy.Publisher("/completed_cloud_vis", PointCloud2, queue_size=1)
            rospy.loginfo("Publishing completed point cloud.")
            rate = rospy.Rate(1)  # 1 Hz
            while not rospy.is_shutdown():
                pub2.publish(res.completed_cloud)
                rate.sleep()
        else:
            rospy.logerr("Shape completion failed (response.success is False).")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
