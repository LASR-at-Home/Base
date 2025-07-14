import rospy
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped

import smach


class SelectObject(smach.State):
    """
    Selects a target object.
    Prioritises graspable objects.
    Choose the closest object within range
    """

    def __init__(self, use_arm: bool = True, range: float = np.inf, base_frame: str = "base_footprint"):
        super().__init__(
            outcomes=["succeeded", "failed"],
            input_keys=["detected_objects"],
            output_keys=["selected_object", "selected_object_name"],
        )
        self._use_arm = use_arm
        self._range = range
        self._base_frame = base_frame


        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def execute(self, userdata):
        rospy.loginfo("Executing object selection")
        userdata.selected_object = None
        userdata.selected_object_name = ""

        if not userdata.detected_objects:
            rospy.logwarn("No detected objects.")
            return "failed"

        closest_obj = None
        closest_dist = self._range

        for obj, pcl in userdata.detected_objects:
            if self._use_arm:
                try:
                    pose = PoseStamped()
                    pose.header = obj.pose.header
                    pose.pose = obj.pose.pose
                    pose_base = self.tf_buffer.transform(pose, self._base_frame, timeout=rospy.Duration(1.0))
                    distance = np.linalg.norm([
                        pose_base.pose.position.x,
                        pose_base.pose.position.y,
                        pose_base.pose.position.z,
                    ])
                    
                    rospy.loginfo(f"Found object '{obj.name}' at distance {distance:.2f}")

                    if distance <= self._range and rospy.get_param(f"/storing_groceries/objects/{obj.name}/graspable", False):
                        if distance < closest_dist:
                            closest_dist = distance
                            closest_obj = (obj, pcl)

                except Exception as e:
                    rospy.logwarn(f"TF transform failed for {obj.name}: {e}")
                    continue
            else:
                closest_obj = (obj, pcl)
                break 

        if closest_obj:
            userdata.selected_object = closest_obj
            userdata.selected_object_name = closest_obj[0].name
            rospy.loginfo(f"Selected object: {closest_obj[0].name}")
            return "succeeded"

        rospy.logwarn("No suitable object found within range.")
        return "failed"
