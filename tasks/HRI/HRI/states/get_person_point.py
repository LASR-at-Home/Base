from smach_ros import RosState


class GetPersonPoint(RosState):
    """State to get the point of interest of the person."""

    def __init__(self, node):
        super().__init__(
            node=node,
            outcomes=["succeeded", "failed"],
            input_keys=["person_detections"],
            output_keys=["person_point"],
        )

    def execute(self, userdata):
        if not userdata.person_detections:
            return "failed"
        # Assuming the first detection is the point of interest
        userdata.person_point = userdata.person_detections[0].point
        return "succeeded"
