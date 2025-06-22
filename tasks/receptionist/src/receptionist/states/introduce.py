"""
State machine that introduces the greeted guest to all other guests/host present in the
seating area.

"""

from typing import Any, Dict, List, Optional

import rospy
import message_filters
import smach

from smach import UserData
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, PointStamped

from lasr_skills import LookToPoint, Say, Wait
from lasr_vision_msgs.srv import Recognise3D, Recognise3DRequest


class Recognise(smach.State):

    _rgb_image: Image
    _depth_image: Image
    _depth_camera_info: CameraInfo
    _rgb_image_topic: str
    _depth_image_topic: str
    _depth_info_topic: str

    def __init__(self):
        super().__init__(
            outcomes=["succeeded", "failed"],
            input_keys=["guest_data", "guest_seat_point", "seated_guest_locs"],
            output_keys=["named_guest_detection"],
        )
        self._rgb_image = None
        self._depth_image = None
        self._depth_camera_info = None

        self._rgb_image_topic = "/xtion/rgb/image_raw"
        self._depth_image_topic = "/xtion/depth_registered/image_raw"
        self._depth_info_topic = "/xtion/depth_registered/camera_info"

    def execute(self, userdata: UserData) -> str:

        recognise = rospy.ServiceProxy("/lasr_vision_reid/recognise", Recognise3D)
        recognise.wait_for_service()

        def get_images_cb(
            image: Image, depth_image: Image, depth_camera_info: CameraInfo
        ) -> None:
            self._rgb_image = image
            self._depth_image = depth_image
            self._depth_camera_info = depth_camera_info

            if (
                self._rgb_image is None
                or self._depth_image is None
                or self._depth_camera_info is None
            ):
                self._rgb_image = image
                self._depth_image = depth_image
                self._depth_camera_info = depth_camera_info

        image_sub = message_filters.Subscriber(self._image_topic, Image)
        depth_sub = message_filters.Subscriber(self._depth_topic, Image)
        depth_camera_info_sub = message_filters.Subscriber(
            self._depth_info_topic, CameraInfo
        )
        ts = message_filters.ApproximateTimeSynchronizer(
            [image_sub, depth_sub, depth_camera_info_sub], 10, 2.0
        )
        ts.registerCallback(get_images_cb)

        while (
            self._rgb_image is None
            or self._depth_image is None
            or self._depth_camera_info is None
        ):
            rospy.sleep(0.05)

        request = Recognise3DRequest(
            image_raw=self._rgb_image,
            depth_image=self._depth_image,
            depth_camera_info=self._depth_camera_info,
            threshold=0.5,
            target_frame="map",
        )
        try:
            response = recognise(request)

            if len(response.detections) == 0:
                rospy.logwarn("No detections found.")
                return "failed"
            else:
                # Assume we have already cropped the image
                userdata.named_guest_detection = response.detections[0]

        except rospy.ServiceException as e:
            rospy.logwarn(f"Unable to perform recognition. ({str(e)})")
            return "failed"

        return "succeeded"


class GetGuestData(smach.State):

    _guest_to_introduce: Optional[str]

    def __init__(self, guest_to_introduce: Optional[str] = None):
        super().__init__(
            outcomes=["succeeded", "failed"],
            input_keys=["guest_data", "named_guest_detection", "person_index"],
            output_keys=["relevant_guest_data", "introduce_to"],
        )
        # If this is None, we assume we have to infer the guest to introduce
        # based on the person_index
        self._guest_to_introduce = guest_to_introduce

    def execute(self, userdata: UserData) -> str:
        if self._guest_to_introduce is not None:
            userdata.relevant_guest_data = userdata.guest_data[self._guest_to_introduce]

            userdata.introduce_to = userdata.named_guest_detection[
                userdata.person_index
            ].name
        else:
            guest_to_introduce_name = userdata.named_guest_detection[
                userdata.person_index
            ].name

            userdata.introduce_to = userdata.guest_data[guest_to_introduce_name]["name"]
            for guest_info in userdata.guest_data.values():
                if guest_info["name"] == guest_to_introduce_name:
                    userdata.relevant_guest_data = guest_info
                    break

        rospy.loginfo(
            f"Introducing {userdata.relevant_guest_data['name']} to {userdata.introduce_to}."
        )

        return "succeeded"


class GetIntroductionStr(smach.State):

    def __init__(self):
        super().__init__(
            outcomes=["succeeded", "failed"],
            input_keys=["relevant_guest_data", "introduce_to"],
            output_keys=["introduction_str"],
        )

    def execute(self, userdata: UserData) -> str:

        guest_to_introduce_data = userdata.relevant_guest_data
        guest_to_introduce_to = userdata.introduce_to

        introduction_str = (
            f"Hello {guest_to_introduce_to}, "
            f"this is {guest_to_introduce_data['name']}. "
            f"Their favourite drink is {guest_to_introduce_data['drink']}, "
            f"and their interest is {guest_to_introduce_data['interest']}."
        )
        userdata.introduction_str = introduction_str
        return "succeeded"


class Introduce(smach.StateMachine):

    _guest_to_introduce: str

    def _get_look_point(self, userdata: smach.UserData) -> str:
        """
        Callback to get the look point based on the current person detection index.

        Args:
            userdata (smach.UserData): User data containing the people detections and index.

        Returns:
            str: Outcome of the state, "succeeded".
        """
        index = userdata.person_index
        if index < len(userdata.seated_guest_locs):
            look_point = PointStamped(
                header=rospy.Header(frame_id="map"),
                point=userdata.seated_guest_locs[index].point,
            )
            userdata.look_point = look_point
            rospy.loginfo(
                f"Look point set to: {userdata.look_point.point.x}, {userdata.look_point.point.y}, {userdata.look_point.point.z}"
            )
            return "succeeded"
        else:
            rospy.logerr("Index out of bounds for people detection points.")
            return "failed"

    def __init__(
        self,
        guest_to_introduce: str,
    ):
        super().__init__(
            outcomes=["succeeded", "failed"],
            input_keys=["guest_data", "guest_seat_point", "seated_guest_locs"],
        )
        self._guest_to_introduce = guest_to_introduce
        with self:
            introduction_iterator = smach.Iterator(
                it=lambda: range(len(self.userdata.seated_guest_locs)),
                it_label="person_index",
                input_keys=["seated_guest_locs", "guest_data", "guest_seat_point"],
                exhausted_outcome="succeeded",
                outcomes=["succeeded", "failed"],
            )
            with introduction_iterator:
                container_sm = smach.StateMachine(
                    outcomes=["continue", "succeeded", "failed"],
                    input_keys=[
                        "guest_data",
                        "guest_seat_point",
                        "seated_guest_locs",
                        "person_index",
                    ],
                )
                with container_sm:
                    smach.StateMachine.add(
                        "GET_LOOK_POINT_1",
                        smach.CBState(
                            self._get_look_point,
                            input_keys=["seated_guest_locs", "person_index"],
                            output_keys=["look_point"],
                        ),
                        transitions={
                            "succeeded": "LOOK_TO_GUEST_1",
                            "failed": "failed",
                        },
                        remapping={"look_point": "pointstamped"},
                    )
                    smach.StateMachine.add(
                        "LOOK_TO_GUEST_1",
                        LookToPoint(),
                        transitions={
                            "succeeded": "WAIT",
                            "aborted": "failed",
                            "timed_out": "failed",
                        },
                    )
                    smach.StateMachine.add(
                        "WAIT",
                        Wait(0.25),
                        transitions={
                            "succeeded": "RECOGNISE",
                            "aborted": "failed",
                            "timed_out": "failed",
                        },
                    )
                    smach.StateMachine.add(
                        "RECOGNISE",
                        Recognise(),
                        transitions={
                            "succeeded": "GET_GUEST_DATA_1",
                            "failed": "failed",
                        },
                        remapping={
                            "guest_data": "guest_data",
                            "guest_seat_point": "guest_seat_point",
                            "named_guest_detection": "named_guest_detection",
                        },
                    )
                    smach.StateMachine.add(
                        "GET_GUEST_DATA_1",
                        GetGuestData(),
                        transitions={
                            "succeeded": "GET_INTRODUCTION_STR_1",
                            "failed": "failed",
                        },
                        remapping={
                            "relevant_guest_data": "relevant_guest_data",
                        },
                    )

                    smach.StateMachine.add(
                        "GET_INTRODUCTION_STR_1",
                        GetIntroductionStr(),
                        transitions={
                            "succeeded": "SAY_INTRODUCTION",
                            "failed": "failed",
                        },
                        remapping={
                            "introduction_str": "text",
                        },
                    )
                    smach.StateMachine.add(
                        "SAY_INTRODUCTION",
                        Say(),
                        transitions={
                            "succeeded": "LOOK_TO_GUEST_2",
                            "aborted": "failed",
                            "preempted": "failed",
                        },
                        remapping={"text": "text"},
                    )
                    smach.StateMachine.add(
                        "LOOK_TO_GUEST_2",
                        LookToPoint(),
                        transitions={
                            "succeeded": "GET_GUEST_DATA_2",
                            "aborted": "failed",
                            "timed_out": "failed",
                        },
                        remapping={"pointstamped": "guest_seat_point"},
                    )
                    smach.StateMachine.add(
                        "GET_GUEST_DATA_2",
                        GetGuestData(guest_to_introduce=self._guest_to_introduce),
                        transitions={
                            "succeeded": "GET_INTRODUCTION_STR_2",
                            "failed": "failed",
                        },
                        remapping={
                            "relevant_guest_data": "relevant_guest_data",
                            "introduce_to": "introduce_to",
                        },
                    )
                    smach.StateMachine.add(
                        "GET_INTRODUCTION_STR_2",
                        GetIntroductionStr(),
                        transitions={
                            "succeeded": "SAY_INTRODUCTION_2",
                            "failed": "failed",
                        },
                        remapping={
                            "introduction_str": "text",
                        },
                    )
                    smach.StateMachine.add(
                        "SAY_INTRODUCTION_2",
                        Say(),
                        transitions={
                            "succeeded": "continue",
                            "aborted": "failed",
                            "preempted": "failed",
                        },
                        remapping={"text": "text"},
                    )
                smach.Iterato.set_contained_state(
                    "CONTAINER_SM",
                    container_sm,
                    loop_outcomes=["continue"],
                )
            self.add(
                "INTRODUCTION_ITERATOR",
                introduction_iterator,
                transitions={
                    "succeeded": "succeeded",
                    "failed": "failed",
                },
            )
