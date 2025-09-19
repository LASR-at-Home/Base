import smach
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point, PointStamped
import cv2_pcl
import cv2_img
from play_motion_msgs.msg import PlayMotionGoal
from collections import Counter
from geometry_msgs.msg import Point, Pose, Quaternion
from shapely.geometry import Point as ShapelyPoint, Polygon
from move_base_msgs.msg import MoveBaseGoal


class CheckOrder(smach.State):
    def __init__(self, context):
        smach.State.__init__(self, outcomes=["correct", "incorrect"])
        self.context = context
        self.n_checks = 0

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
        if self.n_checks == 1: # TODO change back to 3
            self.context.say(
                "I think I have something in my eyes, I'm struggling to check the order. I trust you that the order is correct!"
            )
            self.n_checks = 0
            return "correct"

        self.n_checks += 1

        position = rospy.get_param("/coffee_shop/counter/location/position")
        orientation = rospy.get_param("/coffee_shop/counter/location/orientation")

        move_base_goal = MoveBaseGoal()
        move_base_goal.target_pose.header.frame_id = "map"
        move_base_goal.target_pose.pose = Pose(
            position=Point(**position), orientation=Quaternion(**orientation)
        )
        self.context.move_base_client.send_goal_and_wait(move_base_goal)

        pm_goal = PlayMotionGoal(motion_name="check_table_low", skip_planning=True)
        self.context.play_motion_client.send_goal_and_wait(pm_goal)

        order = self.context.tables[self.context.current_table]["order"]
        counter_corners = rospy.get_param(f"/coffee_shop/counter/cuboid")
        rospy.sleep(rospy.Duration(1.0))
        pcl_msg = rospy.wait_for_message("/xtion/depth_registered/points", PointCloud2)
        cv_im = cv2_pcl.pcl_to_cv2(pcl_msg)
        img_msg = cv2_img.cv2_img_to_msg(cv_im)
        detections = self.context.yolo(img_msg, self.context.YOLO_counter_model, 0.3, []) # TODO check confidence was 0.6 or 0.3
        detections = [
            (det, self.estimate_pose(pcl_msg, det))
            for det in detections.detected_objects
            if det.name in self.context.target_object_remappings.keys()
        ]
        shapely_polygon = Polygon(counter_corners)
        satisfied_points = [
            # shapely_polygon.contains(ShapelyPoint(pose[0], pose[1]))
            True
            for _, pose in detections
        ]
        # given_order = [
        #     detections[i] for i in range(0, len(detections)) if satisfied_points[i]
        # ]
        given_order = []
        for i in range(0, len(detections)):
            if satisfied_points[i]:
                given_order.append(detections[i])
            else:
                rospy.loginfo(f"Filtered out {detections[i][0].name} at position {detections[i][1]} for being outside the counter area.")
                rospy.loginfo(f"Counter area corners: {counter_corners} and polygon: {shapely_polygon}, point: {ShapelyPoint(detections[i][1][0], detections[i][1][1])}")

        rospy.loginfo(detections)
        rospy.loginfo(f"Given order before filtering: {[detection[0].name for detection in given_order]}")
        for _, pose in detections:
            self.context.publish_object_pose(*pose, "map")

        given_order = [detection[0].name for detection in given_order]
        rospy.loginfo(f"Order after detection: {order}, Given order: {given_order}")
        given_order[:] = [x if x != "biscuits" else "granola" for x in given_order]
        rospy.loginfo(f"Order after remapping: {order}, Given order: {given_order}")

        if sorted(order) == sorted(given_order):
            self.n_checks = 0
            return "correct"

        missing_items = list((Counter(order) - Counter(given_order)).elements())
        missing_items_string = ", ".join(
            [
                f"{count} {self.context.target_object_remappings[item] if count == 1 else self.context.target_object_remappings[item]+'s'}"
                for item, count in Counter(missing_items).items()
            ]
        ).replace(", ", ", and ", len(missing_items) - 2)
        invalid_items = list((Counter(given_order) - Counter(order)).elements())
        invalid_items_string = ", ".join(
            [
                f"{count} {self.context.target_object_remappings[item] if count == 1 else self.context.target_object_remappings[item]+'s'}"
                for item, count in Counter(invalid_items).items()
            ]
        ).replace(", ", ", and ", len(invalid_items) - 2)
        rospy.loginfo(f"Order: {order}, Given order: {given_order}")

        if not len(invalid_items):
            self.context.say(
                f"You didn't give me {missing_items_string} which I asked for. Please correct the order."
            )
        elif not len(missing_items):
            self.context.say(
                f"You have given me {invalid_items_string} which I didn't ask for. Please correct the order."
            )
        else:
            self.context.say(
                f"You have given me {invalid_items_string} which I didn't ask for, and didn't give me {missing_items_string} which I asked for. Please correct the order."
            )
        return "incorrect"
