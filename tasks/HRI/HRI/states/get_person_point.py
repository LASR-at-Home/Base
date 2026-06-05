import yasmin


class GetPersonPoint(yasmin.State):
    """State to get the point of interest of the person."""

    def __init__(self):
        super().__init__(
            outcomes=["succeeded", "failed"],
        )
        self.add_input_key('person_detections')
        self.add_output_key('person_point')

    def execute(self, blackboard):
        if not blackboard['person_detections']:
            return "failed"
        # Assuming the first detection is the point of interest
        blackboard['person_point'] = blackboard['person_detections'][0].point
        return "succeeded"
