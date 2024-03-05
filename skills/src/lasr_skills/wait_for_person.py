#!/usr/bin/env python3

import smach

from lasr_skills import Detect


class WaitForPerson(smach.StateMachine):

    class CheckForPerson(smach.State):

        def __init__(self):
            smach.State.__init__(
                self, outcomes=["done", "not_done"], input_keys=["detections"]
            )

        def execute(self, userdata):
            if len(userdata.detections.detected_objects):
                return "done"
            else:
                return "not_done"

    def __init__(self):

        smach.StateMachine.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["depth_topic"],
            output_keys=["detections"],
        )

        with self:
            smach.StateMachine.add(
                "DETECT_PEOPLE",
                Detect(filter=["person"]),
                transitions={"succeeded": "CHECK_FOR_PERSON", "failed": "failed"},
            )
            smach.StateMachine.add(
                "CHECK_FOR_PERSON",
                self.CheckForPerson(),
                transitions={"done": "succeeded", "not_done": "DETECT_PEOPLE"},
            )
