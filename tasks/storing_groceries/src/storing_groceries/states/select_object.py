import rospy

import smach


class SelectObject(smach.State):
    """
    Selects a target object.
    Prioritises graspable objects.
    """

    def __init__(self):
        super().__init__(
            outcomes=["succeeded", "failed"],
            input_keys=["detected_objects"],
            output_keys=["selected_object"],
        )

    def execute(self, userdata):

        if not userdata.detected_objects:
            return "failed"

        for object, pcl in userdata.detected_objects:
            if rospy.get_param(f"/storing_groceries/objects/{object.name}/graspable"):
                userdata.selected_object = (object, pcl)
                return "succeeded"
        return "failed"
