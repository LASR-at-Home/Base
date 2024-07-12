<<<<<<< HEAD
#!/usr/bin/env python3
=======
import smach
>>>>>>> 53286bda68794b7b6e18dde5149dda24bcc473ae
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
<<<<<<< HEAD
            super(QueryImage, self).__init__(
                "/clip_vqa/query_service",
                Vqa,
                request_slots=["possible_answers", "image_raw"],
                response_slots=["answer", "similarity"],
            )


if __name__ == "__main__":
    import rospy
    import smach
    from sensor_msgs.msg import Image

    rospy.init_node("clip_test")

    while True:
        possible_answers = [
            "a person standing up",
            "a person sitting down",
            "a person laying down",
        ]
        image_topic = "/xtion/rgb/image_raw"
        img_msg = rospy.wait_for_message(image_topic, Image)

        sm = smach.StateMachine(outcomes=["succeeded", "failed"])

        with sm:
            sm.userdata.image_raw = img_msg
            sm.userdata.possible_answers = possible_answers
            smach.StateMachine.add(
                "QUERY_IMAGE",
                QueryImage(),
                transitions={
                    "succeeded": "succeeded",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )

        sm.execute()
        input("Press enter to continue...")

    rospy.spin()
=======

            @smach.cb_interface(input_keys=["img_msg"])
            def request_cb(userdata, request):
                return VqaRequest(image=userdata.img_msg)

        super(QueryImage, self).__init__(
            "/clip_vqa/query_service",
            Vqa,
            request_cb=request_cb,
            response_slots=["answer", "similarity"],
        )
>>>>>>> 53286bda68794b7b6e18dde5149dda24bcc473ae
