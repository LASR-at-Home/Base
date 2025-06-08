"""
State for parsing the transcription of the guests' interest, and adding this
to the guest data userdata
"""

import rospy
import smach
from smach import UserData
from typing import List, Dict, Any
from receptionist.states import SpeechRecovery
from lasr_skills import (
    Say,
)


class CompareInterest(smach.StateMachine):
    class FindCommonInterest(smach.State):
        def __init__(self, guest_id: str):
            smach.State.__init__(
                self,
                outcomes=["succeeded", "failed"],
                input_keys=["guest_data", "host_data"],
                output_keys=[
                    "guest_data",
                    "host_data",
                    "common_host_guest1",
                    "common_host_guest2",
                    "common_guest1_guest2",
                ],
            )
            self._guest_id = guest_id

        def execute(self, userdata: UserData) -> str:
            outcome = "succeeded"

            interest_categories = {
                "football": {
                    "football",
                    "sports",
                    "outdoor sports",
                    "team activities",
                    "competitive games",
                    "fitness",
                },
                "basketball": {
                    "basketball",
                    "sports",
                    "outdoor sports",
                    "team activities",
                    "competitive games",
                    "fitness",
                },
                "table tennis": {
                    "table tennis",
                    "sports",
                    "indoor sports",
                    "duo activities",
                    "competitive games",
                },
                "badminton": {
                    "badminton",
                    "sports",
                    "indoor sports",
                    "duo activities",
                    "competitive games",
                },
                "running": {
                    "running",
                    "sports",
                    "individual sports",
                    "outdoor activities",
                    "fitness",
                    "self-challenge",
                },
                "swimming": {
                    "swimming",
                    "sports",
                    "individual sports",
                    "fitness",
                    "outdoor activities",
                    "health",
                },
                "chess": {
                    "chess",
                    "indoor activities",
                    "strategy games",
                    "mind sports",
                },
                "cooking": {
                    "cooking",
                    "indoor activities",
                    "creative activities",
                    "solo activities",
                    "practical skills",
                },
                "painting": {
                    "painting",
                    "indoor activities",
                    "creative activities",
                    "solo activities",
                },
                "reading": {
                    "reading",
                    "indoor activities",
                    "self-learning",
                    "solo activities",
                },
                "yoga": {
                    "yoga",
                    "indoor activities",
                    "fitness",
                    "solo activities",
                    "mental health",
                },
                "robotics": {
                    "robotics",
                    "technology",
                    "engineering",
                    "indoor activities",
                    "problem solving",
                },
                "programming": {
                    "programming",
                    "technology",
                    "indoor activities",
                    "creative activities",
                    "problem solving",
                },
                "electronics": {
                    "electronics",
                    "technology",
                    "engineering",
                    "hands-on activities",
                },
                "travel": {
                    "travel",
                    "outdoor activities",
                    "exploration",
                    "adventure",
                    "cultural exchange",
                },
                "hiking": {
                    "hiking",
                    "outdoor activities",
                    "fitness",
                    "adventure",
                    "self-challenge",
                },
                "camping": {
                    "camping",
                    "outdoor activities",
                    "group activities",
                    "survival skills",
                },
                "dancing": {
                    "dancing",
                    "performance arts",
                    "fitness",
                    "group activities",
                    "creative expression",
                },
                "singing": {
                    "singing",
                    "performance arts",
                    "solo activities",
                    "creative expression",
                },
                "playing guitar": {
                    "playing guitar",
                    "performance arts",
                    "solo activities",
                    "creative expression",
                    "musical skills",
                },
                "photography": {
                    "photography",
                    "outdoor activities",
                    "creative activities",
                    "solo activities",
                    "visual arts",
                },
                "gardening": {
                    "gardening",
                    "outdoor activities",
                    "hobby activities",
                    "solo activities",
                    "relaxation",
                },
                "video games": {
                    "video games",
                    "indoor activities",
                    "competitive games",
                    "technology",
                    "strategy games",
                },
                "board games": {
                    "board games",
                    "indoor activities",
                    "group activities",
                    "strategy games",
                    "entertainment",
                },
            }

            host_interest = userdata.host_data["interest"]
            guest_1_interest = userdata.guest_data[1]["interest"]

            if self._guest_id == 2:
                guest_2_interest = userdata.guest_data[2]["interest"]

            host_categories = set()
            guest_1_categories = set()
            guest_2_categories = set()

            if host_interest in interest_categories:
                host_categories = interest_categories[host_interest]

            if guest_1_interest in interest_categories:
                guest_1_categories = interest_categories[guest_1_interest]

            if self._guest_id == 2 and guest_2_interest in interest_categories:
                guest_2_categories = interest_categories[guest_2_interest]

            common_host_guest1 = host_categories & guest_1_categories
            common_host_guest2 = set()
            common_guest1_guest2 = set()

            if self._guest_id == 2:
                common_host_guest2 = host_categories & guest_2_categories
                common_guest1_guest2 = guest_1_categories & guest_2_categories

            userdata.common_host_guest1 = list(common_host_guest1)
            if self._guest_id == 2:
                userdata.common_host_guest2 = list(common_host_guest2)
                userdata.common_guest1_guest2 = list(common_guest1_guest2)

            return outcome

    class SayCommonInterest(smach.State):
        def __init__(self, guest_id: int):
            smach.State.__init__(
                self,
                outcomes=["succeeded"],
                input_keys=[
                    "common_host_guest1",
                    "common_host_guest2",
                    "common_guest1_guest2",
                ],
            )
            self._guest_id = guest_id

        def execute(self, userdata: UserData) -> str:
            text = ""

            if self._guest_id == 1:
                if userdata.common_host_guest1:
                    categories = ", ".join(userdata.common_host_guest1)
                    text = (
                        f"The host and the first guest share interests in {categories}."
                    )
                else:
                    text = "The host and the first guest have different interests."

            if self._guest_id == 2:
                if userdata.common_host_guest2:
                    categories = ", ".join(userdata.common_host_guest2)
                    text = f"The host and the second guest share interests in {categories}."
                else:
                    text = "The host and the second guest have different interests."

                if userdata.common_guest1_guest2:
                    categories = ", ".join(userdata.common_guest1_guest2)
                    text += f" Also, the two guests share interests in {categories}."

            # Actually say the generated sentence
            say = Say(text=text)
            say.execute(userdata)

            return "succeeded"

    def __init__(self, guest_id: str):
        super().__init__(
            outcomes=[
                "succeeded",
                "failed",
            ],
            input_keys=["guest_data", "host_data"],
        )

        with self:
            smach.StateMachine.add(
                "FindCommonInterest",
                self.FindCommonInterest(guest_id),
                transitions={
                    "succeeded": "SayCommonInterest",
                    "failed": "failed",
                },
            )
            smach.StateMachine.add(
                "SayCommonInterest",
                self.SayCommonInterest(guest_id),
                transitions={
                    "succeeded": "succeeded",
                },
            )
