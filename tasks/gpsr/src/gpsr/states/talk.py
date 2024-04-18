import smach
from lasr_skills import Say
import time
from typing import Dict
import rospy


class GenerateResponse(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["talk_phrase"],
            output_keys=["response"],
        )

        self._possible_responses: Dict[str, str] = self._create_responses()

    def _create_responses(self) -> Dict[str, str]:
        response = {}
        response["something about yourself"] = (
            "I am a Tiago -- a helpful assistive robot developed by PAL Robotics."
        )
        current_time = time.strftime("%H:%M")
        response["the time"] = f"The current time is {current_time}."
        current_day = time.strftime("%A")
        response["what day is today"] = f"Today is {current_day}."
        tomorrow = time.strftime("%A", time.localtime(time.time() + 86400))
        response["what day is tomorrow"] = f"Tomorrow is {tomorrow}."
        response["your teams name"] = "Our team is called LASR."
        response["your teams country"] = "Our team is from the United Kingdom."
        response["your teams affiliation"] = (
            "Our team is affiliated with King's College London."
        )
        day_of_the_week = current_day
        day_of_the_month = time.strftime("%d")
        response["the day of the week"] = f"Today is {day_of_the_week}."

        response["the day of the month"] = (
            f"The day of the month is {day_of_the_month}."
        )
        return response

    def execute(self, userdata):
        try:
            userdata.response = self._possible_responses[userdata.talk_phrase]
        except KeyError:
            rospy.loginfo(
                f"Failed to generate response for {userdata.talk_phrase} as it is not in the list of possible questions."
            )
            return "failed"
        return "succeeded"


# In future we might want to add looking at person talking to the state machine.
class Talk(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed"])

        with self:
            smach.StateMachine.add(
                "GENERATE_RESPONSE",
                GenerateResponse(),
                transitions={"succeeded": "SAY_RESPONSE", "failed": "failed"},
                remapping={"talk_phrase": "talk_phrase", "response": "response"},
            )

            smach.StateMachine.add(
                "SAY_RESPONSE",
                Say(),
                transitions={"succeeded": "succeeded", "failed": "failed"},
                remapping={"phrase": "response"},
            )
