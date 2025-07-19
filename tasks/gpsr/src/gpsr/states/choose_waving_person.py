import smach


class ChooseWavingPerson(smach.State):
    """
    State to choose a waving person from the list of detected people.
    The last person in the list is chosen
    """

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["hands_up_detections"],
            output_keys=["waving_person_detection", "hands_up_detections"],
        )

    def execute(self, userdata):
        if userdata.hands_up_detections:
            userdata.waving_person_detection = userdata.hands_up_detections.pop()
            return "succeeded"
        else:
            return "failed"
