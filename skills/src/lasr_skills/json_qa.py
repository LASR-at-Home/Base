#!/usr/bin/env python3
import rospy
import smach
import json
from lasr_vector_databases_msgs.srv import TxtQuery, TxtQueryRequest


class JsonQuestionAnswer(smach.State):
    def __init__(self, index_path: str, txt_path: str, json_path: str, k: int = 1):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["query_sentence", "k"],
            output_keys=["closest_answers"],
        )
        self.index_path = index_path
        self.txt_path = txt_path
        self.json_path = json_path
        self.k = k
        self.txt_query = rospy.ServiceProxy("/lasr_faiss/txt_query", TxtQuery)

    def execute(self, userdata):
        rospy.wait_for_service("/lasr_faiss/txt_query")
        q_a_dict = json.load(open(self.json_path, "r"))
        try:
            request = TxtQueryRequest(
                self.txt_path,
                self.index_path,
                userdata.query_sentence,
                self.k,
            )
            result = self.txt_query(request)
            answers = [q_a_dict[question] for question in result.closest_sentences]
            userdata.closest_answers = answers
            return "succeeded"
        except rospy.ServiceException as e:
            return "failed"
