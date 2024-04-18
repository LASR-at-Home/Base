import smach
import rospy
import rospkg
import os
from lasr_vector_databases_msgs.srv import TxtQuery, TxtQueryRequest


class CommandSimilarityMatcher(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["success"],
            input_keys=["command"],
            output_keys=["matched_command"],
        )

        self._query_service = rospy.ServiceProxy("lasr_faiss/txt_query", TxtQuery)
        self._text_directory = os.path.join(
            rospkg.RosPack().get_path("gpsr"), "data", "command_data"
        )
        self._index_directory = os.path.join(
            rospkg.RosPack().get_path("gpsr"), "data", "index_data"
        )
        self._text_paths = [
            os.path.join(self._text_directory, f"all_gpsr_commands_chunk_{i+1}.txt")
            for i in range(10)
        ]
        self._index_paths = [
            os.path.join(self._index_directory, f"all_gpsr_commands_chunk_{i+1}.index")
            for i in range(10)
        ]

    def execute(self, userdata):
        request = TxtQueryRequest()
        request.txt_paths = self._text_paths
        request.index_paths = self._index_paths
        request.query_sentence = userdata.command
        request.k = 1
        response = self._query_service(request)
        userdata.matched_command = response.closest_sentences[0]
        return "success"
