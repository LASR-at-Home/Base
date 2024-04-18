import smach_ros
from lasr_vision_msgs.srv import Vqa, VqaRequest

from typing import List, Union


class QueryImage(smach_ros.ServiceState):

    def __init__(self, possible_answers: Union[None, List[str]] = None):

        if possible_answers is not None:
            super(QueryImage, self).__init__(
                "/clip_vqa/query_service",
                Vqa,
                request=VqaRequest(possible_answers=possible_answers),
                response_slots=["answer", "similarity"],
            )
        else:
            super(QueryImage, self).__init__(
                "/clip_vqa/query_service",
                Vqa,
                request_slots=["possible_answers"],
                response_slots=["answer", "similarity"],
            )
