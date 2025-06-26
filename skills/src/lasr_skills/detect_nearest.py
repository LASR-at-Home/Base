import smach
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped


class GetNearestObject(smach.State):
    """State to process detections and return the nearest object

    Assumes that the point of detected objects are in the map frame.
    """

    _just_point: bool

    def __init__(self, just_point: bool = True):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["detections_3d"],
            output_keys=["nearest_object"],
        )
        self._just_point = just_point

    def execute(self, userdata):
        if not userdata.detections_3d:
            return "failed"

        robot_pose = rospy.wait_for_message("/amcl_pose", PoseWithCovarianceStamped)
        robot_position = robot_pose.pose.pose.position

        closest_distance = float("inf")
        closest_object = None

        for obj in userdata.detections_3d.detected_objects:
            obj_position = obj.point
            distance = (
                (obj_position.x - robot_position.x) ** 2
                + (obj_position.y - robot_position.y) ** 2
                + (obj_position.z - robot_position.z) ** 2
            ) ** 0.5

            if distance < closest_distance:
                closest_distance = distance
                closest_object = obj

        if closest_object is None:
            return "failed"

        if self._just_point:
            userdata.nearest_object = closest_object.point
        else:
            userdata.nearest_object = closest_object
        rospy.loginfo(f"Nearest object found: {closest_object}")
        rospy.loginfo(f"Distance to nearest object: {closest_distance:.2f} meters")

        return "succeeded"
