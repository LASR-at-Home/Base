import smach
import rospy

from std_msgs.msg import String
from play_motion_msgs.msg import PlayMotionGoal
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped, Point
from common_math import pcl_msg_to_cv2, seg_to_centroid
import numpy as np
from shapely.geometry import Point as ShapelyPoint
from shapely.geometry.polygon import Polygon


class PreCheckTable(smach.State):
    def __init__(self, context):
        smach.State.__init__(self, outcomes=["done"])
        self.context = context
        self.detections_people = []

    def estimate_pose(self, pcl_msg, detection):
        centroid_xyz = seg_to_centroid(pcl_msg, np.array(detection.xyseg))
        centroid = PointStamped()
        centroid.point = Point(*centroid_xyz)
        centroid.header = pcl_msg.header
        centroid = self.context.tf_pose(centroid, "map")
        return np.array(
            [
                centroid.target_point.point.x,
                centroid.target_point.point.y,
                centroid.target_point.point.z,
            ]
        )

    def publish_object_points(self):
        for _, point in self.detections_objects:
            self.context.publish_object_pose(*point, "map")

    def publish_people_points(self):
        for _, point in self.detections_people:
            self.context.publish_person_pose(*point, "map")

    def filter_detections_by_pose(self, detections, threshold=0.2):
        filtered = []

        for i, (detection, point) in enumerate(detections):
            distances = np.array(
                [np.sqrt(np.sum((point - ref_point) ** 2)) for _, ref_point in filtered]
            )
            if not np.any(distances < threshold):
                filtered.append((detection, point))

        return filtered

    def perform_detection(self, pcl_msg, polygon, filter, model):
        cv_im = pcl_msg_to_cv2(pcl_msg)
        img_msg = self.context.bridge.cv2_to_imgmsg(cv_im)
        detections = self.context.yolo(img_msg, model, 0.5, 0.3)
        detections = [
            (det, self.estimate_pose(pcl_msg, det))
            for det in detections.detected_objects
            if det.name in filter
        ]
        rospy.loginfo(f"All: {[(det.name, pose) for det, pose in detections]}")
        rospy.loginfo(f"Boundary: {polygon}")
        shapely_polygon = Polygon(polygon)
        satisfied_points = [
            shapely_polygon.contains(ShapelyPoint(pose[0], pose[1]))
            for _, pose in detections
        ]
        detections = [
            detections[i] for i in range(0, len(detections)) if satisfied_points[i]
        ]
        rospy.loginfo(f"Filtered: {[(det.name, pose) for det, pose in detections]}")
        return detections

    def check(self, pcl_msg):
        self.check_people(pcl_msg)

    def check_people(self, pcl_msg):
        detections_people_ = self.perform_detection(
            pcl_msg, self.person_polygon, ["person"], self.context.YOLO_person_model
        )
        self.detections_people.extend(detections_people_)

    def execute(self, userdata):
        self.context.stop_head_manager("head_manager")

        self.context.voice_controller.async_tts("I'm taking a quick look at the table")

        rospy.loginfo(self.context.current_table)
        self.person_polygon = rospy.get_param(
            f"/tables/{self.context.current_table}/persons_cuboid"
        )
        self.detections_people = []

        motions = ["back_to_default", "look_left", "look_right", "back_to_default"]
        for motion in motions:
            pm_goal = PlayMotionGoal(motion_name=motion, skip_planning=True)
            self.context.play_motion_client.send_goal_and_wait(pm_goal)
            pcl_msg = rospy.wait_for_message(
                "/xtion/depth_registered/points", PointCloud2
            )
            self.check(pcl_msg)

        self.detections_people = self.filter_detections_by_pose(
            self.detections_people, threshold=0.50
        )

        people_count = len(self.detections_people)
        people_text = "person" if people_count == 1 else "people"

        self.context.voice_controller.async_tts(
            f"I saw {people_count} {people_text} so far"
        )

        self.context.tables[self.context.current_table][
            "pre_people"
        ] = self.detections_people
        self.context.start_head_manager("head_manager", "")
        return "done"
