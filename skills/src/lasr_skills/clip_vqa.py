#!/usr/bin/env python3

import smach
from smach import UserData
from typing import Union
from lasr_skills import Say
from lasr_vision_clip.srv import VqaRequest, VqaResponse, Vqa


class QueryImage(smach.State):
    def __init__(
        self,
        model_device: str = "cuda",
    ):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["question", "answers"],
            output_keys=["answer", "similarity_score"],
        )
        self._service_proxy = rospy.ServiceProxy("/clip_vqa/query_service", Vqa)

    def execute(self, userdata: UserData):
        answers = userdata.answers
        request = VqaRequest()
        request.possible_answers = answers
        response = self._service_proxy(request)
        userdata.answer = response.answer
        userdata.similarity_score = response.similarity
        return "succeeded"


if __name__ == "__main__":
    import rospy

    rospy.init_node("test_query_image")
    sm = smach.StateMachine(outcomes=["succeeded", "failed"])
    with sm:
        smach.StateMachine.add(
            "QUERY_IMAGE",
            QueryImage(),
            transitions={
                "succeeded": "SAY",
                "failed": "failed",
            },
            remapping={
                "answers": "answers",
                "answer": "answer",
                "similarity_score": "similarity_score",
            },
        )
        smach.StateMachine.add(
            "SAY",
            Say(format_str="I think the answer is {}"),
            transitions={
                "succeeded": "succeeded",
                "aborted": "failed",
                "preempted": "failed",
            },
            remapping={
                "placeholders": "answer",
            },
        )
    sm.userdata.answers = ["a person wearing glasses", "a person not wearing glasses"]
    sm.execute()
