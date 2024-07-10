#!/usr/bin/env python3
import os
import rospy
import rospkg
import smach
from lasr_skills import AskAndListen, Say
from lasr_vector_databases_msgs.srv import TxtQuery, TxtQueryRequest


class MatchNameAndDrink(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["sequence"],
            output_keys=["matched_name", "matched_drink", "sequence"],
        )

        self._query_service = rospy.ServiceProxy("lasr_faiss/txt_query", TxtQuery)
        self._text_path = os.path.join(
            rospkg.RosPack().get_path("receptionist"),
            "data",
            "name_and_drink_vector_db.txt",
        )
        self._index_path = os.path.join(
            rospkg.RosPack().get_path("receptionist"), "data", "name_and_drink.index"
        )

    def execute(self, userdata):
        rospy.loginfo(f"Received transcript: {userdata.sequence}")
        request = TxtQueryRequest()
        request.txt_paths = [self._text_path]
        request.index_paths = [self._index_path]
        request.query_sentence = userdata.sequence
        request.k = 1
        response = self._query_service(request)
        matched_name, matched_drink = response.closest_sentences[0].split(
            " and my favorite drink is "
        )
        matched_name = matched_name.split("My name is ")[1]
        userdata.matched_name = matched_name
        userdata.matched_drink = matched_drink
        rospy.loginfo(
            f"Matched name: {matched_name} and matched drink: {matched_drink}"
        )
        userdata.sequence = (
            f"Hello {matched_name}, I see that your favorite drink is {matched_drink}."
        )
        return "succeeded"


if __name__ == "__main__":
    rospy.init_node("match_name_and_drink")
    sm = smach.StateMachine(outcomes=["succeeded", "failed"])
    with sm:
        smach.StateMachine.add(
            "ASK_FOR_NAME_AND_DRINK",
            AskAndListen(
                tts_phrase="Hello, please tell me your name and favorite drink."
            ),
            transitions={"succeeded": "MATCH_NAME_AND_DRINK", "failed": "failed"},
            remapping={"transcribed_speech": "sequence"},
        )
        smach.StateMachine.add(
            "MATCH_NAME_AND_DRINK",
            MatchNameAndDrink(),
            transitions={"succeeded": "SAY_MATCHED_NAME_AND_DRINK", "failed": "failed"},
            remapping={"sequence": "sequence"},
        )
        smach.StateMachine.add(
            "SAY_MATCHED_NAME_AND_DRINK",
            Say(),
            transitions={
                "succeeded": "succeeded",
                "aborted": "failed",
                "preempted": "failed",
            },
            remapping={
                "text": "sequence",
            },
        )
    sm.execute()
