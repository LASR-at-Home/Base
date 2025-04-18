import smach
from lasr_skills import Say
import time
from typing import Dict
import rospy


# In future we might want to add looking at person talking to the state machine.
class Talk(smach.StateMachine):
    class GenerateResponse(smach.State):
        def __init__(self, talk_phrase: str):
            smach.State.__init__(
                self,
                outcomes=["succeeded", "failed"],
                output_keys=["response"],
            )
            self._talk_phrase = talk_phrase

        def _create_responses(self) -> Dict[str, str]:
            response = {}
            response["something about yourself"] = (
                "I am a Tiago -- a helpful assistive robot developed by PAL Robotics."
            )
            current_time = time.strftime("%H:%M")
            response["the_time"] = f"The current time is {current_time}."
            current_day = time.strftime("%A")
            response["what_day_is_today"] = f"Today is {current_day}."
            tomorrow = time.strftime("%A", time.localtime(time.time() + 86400))
            response["what_day_is_tomorrow"] = f"Tomorrow is {tomorrow}."
            response["your_teams_name"] = "Our team is called LASR."
            response["your_teams_country"] = "Our team is from the United Kingdom."
            response["your_teams_affiliation"] = (
                "Our team is affiliated with King's College London."
            )
            day_of_the_week = current_day
            day_of_the_month = time.strftime("%d")
            response["the_day_of_the_week"] = f"Today is {day_of_the_week}."

            response["the_day_of_the_month"] = (
                f"The_day_of_the_month_is {day_of_the_month}."
            )
            return response

        def execute(self, userdata):
            try:
                userdata.response = self._create_responses()[self._talk_phrase]
            except KeyError:
                rospy.loginfo(
                    f"Failed to generate response for {self._talk_phrase} as it is not in the list of possible questions."
                )
                return "failed"
            return "succeeded"

    def __init__(self, talk_phrase: str):
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed"])

        with self:
            smach.StateMachine.add(
                "GENERATE_RESPONSE",
                self.GenerateResponse(talk_phrase),
                transitions={"succeeded": "SAY_RESPONSE", "failed": "failed"},
                remapping={"response": "response"},
            )

            smach.StateMachine.add(
                "SAY_RESPONSE",
                Say(),
                transitions={
                    "succeeded": "succeeded",
                    "aborted": "failed",
                    "preempted": "failed",
                },
                remapping={"text": "response"},
            )
