import rospy
import numpy as np

import smach


class SelectObject(smach.State):
    """
    Selects a target object.
    Prioritises graspable objects.
    Choose the closest object within range
    """

    def __init__(self, use_arm: bool = True, range: float = np.inf):
        super().__init__(
            outcomes=["succeeded", "failed"],
            input_keys=["detected_objects"],
            output_keys=["selected_object", "selected_object_name"],
        )
        self._use_arm = use_arm

    def execute(self, userdata):

        userdata.selected_object = None
        userdata.selected_object_name = ""

        if not userdata.detected_objects:
            return "failed"

        if self._use_arm:
            for object, pcl in userdata.detected_objects:
                if rospy.get_param(
                    f"/storing_groceries/objects/{object.name}/graspable"
                ):
                    userdata.selected_object = (object, pcl)
                    userdata.selected_object_name = object.name
                    return "succeeded"
            return "failed"
        else:
            userdata.selected_object = userdata.detected_objects[0]
            userdata.selected_object_name = userdata.detected_objects[0][0].name
            return "succeeded"
