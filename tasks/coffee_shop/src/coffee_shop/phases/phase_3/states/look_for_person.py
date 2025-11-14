import smach
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point, PointStamped
import cv2_pcl
import cv2_img
from play_motion_msgs.msg import PlayMotionGoal
from shapely.geometry import Point as ShapelyPoint, Polygon


class LookForPerson(smach.State):
    def __init__(self, context):
        smach.State.__init__(self, outcomes=["found", "not found"])
        self.context = context

    def estimate_pose(self, pcl_msg, detection):
        centroid_xyz = cv2_pcl.seg_to_centroid(pcl_msg, np.array(detection.xyseg))
        centroid = PointStamped()
        centroid.point = Point(*centroid_xyz)
        centroid.header = pcl_msg.header
        centroid = self.context.tf_point(centroid, "map")
        return np.array(
            [
                centroid.point.x,
                centroid.point.y,
                centroid.point.z,
            ]
        )

    def execute(self, userdata):
        self.context.stop_head_manager("head_manager")

        pm_goal = PlayMotionGoal(motion_name="back_to_default", skip_planning=True)
        self.context.play_motion_client.send_goal_and_wait(pm_goal)

        corners = rospy.get_param("/coffee_shop/wait/cuboid")
        pcl_msg = rospy.wait_for_message("/xtion/depth_registered/points", PointCloud2)
        cv_im = cv2_pcl.pcl_to_cv2(pcl_msg)
        img_msg = cv2_img.cv2_img_to_msg(cv_im)
        detections = self.context.yolo(img_msg, self.context.YOLO_person_model, 0.3, [])
        detections = [
            (det, self.estimate_pose(pcl_msg, det))
            for det in detections.detected_objects
            if det.name == "person"
        ]
        shapely_polygon = Polygon(corners)
        satisfied_points = [
            shapely_polygon.contains(ShapelyPoint(pose[0], pose[1]))
            for _, pose in detections
        ]
        if len(detections):
            for i in range(0, len(detections)):
                pose = detections[i][1]
                self.context.publish_person_pose(*pose, "map")
                if satisfied_points[i]:
                    self.context.new_customer_pose = PointStamped(point=Point(*pose))
                    self.context.new_customer_pose.header.frame_id = "map"

                    return "found"
        rospy.sleep(rospy.Duration(1.0))

        self.context.start_head_manager("head_manager", "")

        return "not found"
