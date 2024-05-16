import smach
import rospy

from play_motion_msgs.msg import PlayMotionGoal
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped, Point
import cv2_pcl
import cv2_img
import numpy as np

from shapely.geometry import Point as ShapelyPoint
from shapely.geometry.polygon import Polygon


class CheckTable(smach.State):
    def __init__(self, context):
        smach.State.__init__(self, outcomes=["not_finished", "finished"])
        self.context = context
        self.detections_objects = []
        self.detections_people = []
        self.biscuits = False

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
        cv_im = cv2_pcl.pcl_to_cv2(pcl_msg)
        img_msg = cv2_img.cv2_img_to_msg(cv_im)
        detections = self.context.yolo(img_msg, model, 0.6, 0.3)
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
        self.check_table(pcl_msg)
        self.check_people(pcl_msg)

    def check_table(self, pcl_msg):
        target_objects = list(self.context.target_object_remappings.keys())
        if not self.biscuits and "biscuits" in target_objects:
            target_objects.remove("biscuits")
        detections_objects_ = self.perform_detection(
            pcl_msg,
            self.object_polygon,
            target_objects,
            self.context.YOLO_objects_model,
        )
        self.detections_objects.extend(detections_objects_)

    def check_people(self, pcl_msg):
        detections_people_ = self.perform_detection(
            pcl_msg, self.person_polygon, ["person"], self.context.YOLO_person_model
        )
        self.detections_people.extend(detections_people_)

    def execute(self, userdata):
        self.context.stop_head_manager("head_manager")

        self.context.voice_controller.async_tts("I am going to check the table")
        self.object_debug_images = []
        self.people_debug_images = []

        rospy.loginfo(self.context.current_table)
        self.object_polygon = rospy.get_param(
            f"/tables/{self.context.current_table}/objects_cuboid"
        )
        self.person_polygon = rospy.get_param(
            f"/tables/{self.context.current_table}/persons_cuboid"
        )
        self.detections_objects = []
        self.detections_people = []

        motions = [
            "back_to_default",
            "check_table",
            "check_table_low",
            "look_left",
            "look_right",
            "back_to_default",
        ]

        for motion in motions:
            pm_goal = PlayMotionGoal(motion_name=motion, skip_planning=True)
            self.context.play_motion_client.send_goal_and_wait(pm_goal)
            pcl_msg = rospy.wait_for_message(
                "/xtion/depth_registered/points", PointCloud2
            )
            self.check(pcl_msg)

        status = "unknown"
        if len(self.detections_objects) > 0 and len(self.detections_people) == 0:
            status = "needs cleaning"
        elif len(self.detections_objects) > 0 and len(self.detections_people) > 0:
            status = "served"
        elif len(self.detections_objects) == 0 and len(self.detections_people) > 0:
            status = "needs serving"
        elif len(self.detections_objects) == 0 and len(self.detections_people) == 0:
            status = "ready"

        self.detections_people = self.filter_detections_by_pose(
            self.detections_people, threshold=0.6
        )

        self.publish_object_points()
        self.publish_people_points()

        self.context.tables[self.context.current_table]["status"] = status
        self.context.tables[self.context.current_table]["people"] = [
            pose for _, pose in self.detections_people
        ]

        people_count = len(self.detections_people)
        people_text = "person" if people_count == 1 else "people"
        status_text = f"The status of this table is {status}."
        count_text = f"There {'is' if people_count == 1 else 'are'} {people_count} {people_text}."
        self.context.voice_controller.sync_tts(f"{status_text} {count_text}")

        self.context.start_head_manager("head_manager", "")
        return (
            "not_finished"
            if len(
                [
                    (label, table)
                    for label, table in self.context.tables.items()
                    if table["status"] == "unvisited"
                ]
            )
            else "finished"
        )
