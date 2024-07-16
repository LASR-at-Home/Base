#!/usr/bin/env python3
import smach
import rospy
import rospkg
import os
from lasr_vector_databases_msgs.srv import TxtQuery, TxtQueryRequest


class CommandSimilarityMatcher(smach.State):
    def __init__(self, n_vecs_per_txt_file):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["command", "n_vecs_per_txt_file"],
            output_keys=["matched_command"],
        )
        self._n_vecs_per_txt_file = n_vecs_per_txt_file

        self._query_service = rospy.ServiceProxy("lasr_faiss/txt_query", TxtQuery)
        self._text_directory = os.path.join(
            rospkg.RosPack().get_path("gpsr"), "data", "command_data"
        )
        self._index_directory = os.path.join(
            rospkg.RosPack().get_path("gpsr"), "data", "faiss_indices"
        )
        self._text_paths = [
            os.path.join(self._text_directory, f"all_gpsr_commands_chunk_{i+1}.txt")
            for i in range(10)
        ]
        self._index_paths = [
            os.path.join(self._index_directory, f"all_gpsr_commands.index")
        ]

    def execute(self, userdata):
        rospy.loginfo(f"Received command: {userdata.command}")
        request = TxtQueryRequest()
        request.txt_paths = self._text_paths
        request.index_paths = self._index_paths
        request.query_sentence = userdata.command
        request.k = 1
        request.vecs_per_txt_file = self._n_vecs_per_txt_file
        response = self._query_service(request)
        userdata.matched_command = response.closest_sentences[0]
        rospy.loginfo(f"Matched command: {response.closest_sentences[0]}")
        return "succeeded"


if __name__ == "__main__":
    rospy.init_node("command_similarity_matcher")
    from lasr_skills import AskAndListen, Say, Listen

    sm = smach.StateMachine(outcomes=["succeeded", "failed"])
    with sm:
        sm.userdata.tts_phrase = "Please tell me your command."
        # smach.StateMachine.add(
        #     "ASK_FOR_COMMAND",
        #     AskAndListen(),
        #     transitions={"succeeded": "COMMAND_SIMILARITY_MATCHER", "failed": "failed"},
        #     remapping={"transcribed_speech": "command"},
        # )
        sm.add(
            "LISTEN",
            Listen(),
            transitions={
                "succeeded": "COMMAND_SIMILARITY_MATCHER",
                "preempted": "failed",
                "aborted": "failed",
            },
            remapping={"sequence": "command"},
        )
        sm.add(
            "COMMAND_SIMILARITY_MATCHER",
            CommandSimilarityMatcher([840342] * 10),
            transitions={"succeeded": "LISTEN", "failed": "failed"},
        )
        # sm.add(
        #     "SAY_MATCHED_COMMAND",
        #     Say(),
        #     transitions={
        #         "succeeded": "ASK_FOR_COMMAND",
        #         "aborted": "failed",
        #         "preempted": "failed",
        #     },
        #     remapping={"text": "matched_command"},
        # )

    sm.execute()
    rospy.spin()
