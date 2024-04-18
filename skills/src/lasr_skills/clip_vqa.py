import smach_ros
from lasr_vision_clip.srv import Vqa


class QueryImage(smach_ros.ServiceState):

    def __init__(
        self,
    ):
        super(smach_ros.ServiceState, self).__init__(
            "/clip_vqa/query_service",
            Vqa,
            request_slots=["answers"],
            response_slots=["answer", "similarity_score"],
        )
