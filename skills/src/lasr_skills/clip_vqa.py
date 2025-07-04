import smach
import smach_ros
from lasr_vision_msgs.srv import Vqa, VqaRequest
from typing import List, Optional, Union
 




class QueryImage(smach_ros.ServiceState):
    def __init__(self, possible_answers: Optional[List[str]] = None):
        @smach.cb_interface(
            input_keys=["img_msg"],
            output_keys=["answer", "similarity"]  
        )
        def request_cb(userdata, request):
            return VqaRequest(
                image_raw=userdata.img_msg,
                possible_answers=possible_answers or [],
            )
        
        super(QueryImage, self).__init__(
            "/clip_vqa/query_service",
            Vqa,
            request_cb=request_cb,
            response_slots=["answer", "similarity"],
        )
