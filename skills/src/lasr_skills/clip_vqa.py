import smach
import smach_ros
from lasr_vision_msgs.srv import Vqa, VqaRequest

from typing import List, Union


class QueryImage(smach_ros.ServiceState):
    def __init__(self, possible_answers: Union[None, List[str]] = None):

        if possible_answers is not None:

            @smach.cb_interface(input_keys=["img_msg"])
            def request_cb(userdata, request):
                return VqaRequest(
                    possible_answers=possible_answers, image=userdata.img_msg
                )

        else:

            @smach.cb_interface(input_keys=["img_msg"])
            def request_cb(userdata, request):
                return VqaRequest(image=userdata.img_msg)

        super(QueryImage, self).__init__(
            "/clip_vqa/query_service",
            Vqa,
            request_cb=request_cb,
            response_slots=["answer", "similarity"],
        )
