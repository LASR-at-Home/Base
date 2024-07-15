#!/usr/bin/env python3
import os
import cv2
import rospy
import smach
import cv2_img
import numpy as np

from lasr_vision_msgs.srv import (
    YoloDetection,
    BodyPixMaskDetection,
    BodyPixMaskDetectionRequest,
    TorchFaceFeatureDetectionDescription,
    Vqa,
    VqaRequest,
)
from numpy2message import numpy2message
from .vision import GetCroppedImage, ImageMsgToCv2
import numpy as np
from lasr_skills.validate_keypoints import ValidateKeypoints


class DescribePeople(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=[],
            output_keys=["people"],
        )

        with self:
            smach.StateMachine.add(
                "GET_IMAGE",
                GetCroppedImage(
                    object_name="person",
                    method="closest",
                    use_mask=False,  # If true prediction can be very wrong!!!
                ),
                transitions={
                    "succeeded": "CLIP ATTRIBUTES",
                    "failed": "failed",
                },
            )

            smach.StateMachine.add(
                self.GetClipAttributes(),
                transitions={"succeeded": "CONVERT_IMAGE", "failed": "failed"},
            )

            smach.StateMachine.add(
                "CONVERT_IMAGE", ImageMsgToCv2(), transitions={"succeeded": "SEGMENT"}
            )

            sm_con = smach.Concurrence(
                outcomes=["succeeded", "failed"],
                default_outcome="failed",
                outcome_map={
                    "succeeded": {
                        "SEGMENT_YOLO": "succeeded",
                        "SEGMENT_BODYPIX": "succeeded",
                    }
                },
                input_keys=[
                    "img",
                    "img_msg",
                ],
                output_keys=["people_detections", "bodypix_masks"],
            )

            with sm_con:
                smach.Concurrence.add("SEGMENT_YOLO", self.SegmentYolo())
                smach.Concurrence.add("SEGMENT_BODYPIX", self.SegmentBodypix())

            smach.StateMachine.add(
                "SEGMENT", sm_con, transitions={"succeeded": "FEATURE_EXTRACTION"}
            )
            smach.StateMachine.add(
                "FEATURE_EXTRACTION",
                self.FeatureExtraction(),
                transitions={"succeeded": "succeeded"},
            )

    class GetClipAttributes(smach.State):
        def __init__(self):
            smach.State.__init__(
                self,
                outcomes=["succeeded", "failed"],
                input_keys=["image_raw"],
                output_keys=["clip_detection_dict"],
            )
            self.clip_service = rospy.ServiceProxy("/clip_vqa/query_service", Vqa)
            self.glasses_questions = [
                "a person wearing glasses",
                "a person not wearing glasses",
            ]
            self.hat_questions = [
                "a person wearing a hat",
                "a person not wearing a hat",
            ]
            self.hair_questions = [
                "a person with long hair",
                "a person with short hair",
            ]

        def execute(self, userdata):
            try:
                glasses_request = VqaRequest(
                    possible_answers=self.glasses_questions,
                    image_raw=userdata.image_raw,
                )
                glasses_response = self.clip_service(glasses_request)
                hat_request = VqaRequest(
                    possible_answers=self.hat_questions, image_raw=userdata.image_raw
                )
                hat_response = self.clip_service(hat_request)
                hair_request = VqaRequest(
                    possible_answers=self.hair_questions, image_raw=userdata.image_raw
                )
                hair_response = self.clip_service(hair_request)

                glasses_bool = glasses_response.answer == "a person wearing glasses"
                hat_bool = hat_response.answer == "a person wearing a hat"
                hair_bool = hair_response.answer == "a person with long hair"

                userdata.clip_detection_dict = {
                    "glasses": glasses_bool,
                    "hat": hat_bool,
                    "long_hair": hair_bool,
                }
            except Exception as e:
                rospy.logerr(f"Failed to get clip attributes: {e}")
                return "failed"
            return "succeeded"

    class SegmentYolo(smach.State):
        """
        Segment using YOLO

        This should be turned into / merged with generic states
        """

        def __init__(self):
            smach.State.__init__(
                self,
                outcomes=["succeeded", "failed"],
                input_keys=["img_msg"],
                output_keys=["people_detections"],
            )
            self.yolo = rospy.ServiceProxy("/yolov8/detect", YoloDetection)

        def execute(self, userdata):
            try:
                result = self.yolo(userdata.img_msg, "yolov8x-seg.pt", 0.5, 0.3)
                userdata.people_detections = [
                    det for det in result.detected_objects if det.name == "person"
                ]
                return "succeeded"
            except rospy.ServiceException as e:
                rospy.logwarn(f"Unable to perform inference. ({str(e)})")
                return "failed"

    class SegmentBodypix(smach.State):
        """
        Segment using Bodypix

        This should be turned into / merged with generic states
        """

        def __init__(self):
            smach.State.__init__(
                self,
                outcomes=["succeeded", "failed"],
                input_keys=[
                    "img_msg",
                ],
                output_keys=["bodypix_masks"],
            )
            self.bodypix = rospy.ServiceProxy(
                "/bodypix/mask_detection", BodyPixMaskDetection
            )

        def execute(self, userdata):
            try:
                request = BodyPixMaskDetectionRequest()
                request.image_raw = userdata.img_msg
                request.dataset = "resnet50"
                request.confidence = 0.2
                request.parts = ["torso_front", "torso_back", "left_face", "right_face"]
                result = self.bodypix(request)
                userdata.bodypix_masks = result.masks
                return "succeeded"
            except rospy.ServiceException as e:
                rospy.logerr(f"Unable to perform inference. ({str(e)})")
                return "failed"

    class FeatureExtraction(smach.State):
        """
        Perform feature extraction

        This could be split into more states in theory, but that might just be unnecessary work
        """

        def __init__(self):
            smach.State.__init__(
                self,
                outcomes=["succeeded", "failed"],
                input_keys=["img", "people_detections", "bodypix_masks"],
                output_keys=["people"],
            )
            self.face_features = rospy.ServiceProxy(
                "/torch/detect/face_features", TorchFaceFeatureDetectionDescription
            )

        def execute(self, userdata):
            if len(userdata.people_detections) == 0:
                rospy.logerr("Couldn't find anyone!")
                userdata.people = []
                return "failed"
            elif len(userdata.people_detections) == 1:
                rospy.logdebug("There is one person.")
            else:
                rospy.logdebug(f"There are {len(userdata.people_detections)} people.")

            img = userdata.img
            height, width, _ = img.shape

            people = []
            for person in userdata.people_detections:
                rospy.logdebug(f"\n\nFound person with confidence {person.confidence}!")

                # mask for this person
                mask_image = np.zeros((height, width), np.uint8)
                contours = np.array(person.xyseg).reshape(-1, 2)
                cv2.fillPoly(
                    mask_image, pts=np.int32([contours]), color=(255, 255, 255)
                )
                mask_bin = mask_image > 0
                torso_mask = np.zeros((height, width), np.uint8)
                head_mask = np.zeros((height, width), np.uint8)
                # process part masks
                for part in userdata.bodypix_masks:
                    part_mask = np.array(part.mask).reshape(
                        part.shape[0], part.shape[1]
                    )

                    # filter out part for current person segmentation
                    try:
                        part_mask[mask_bin == 0] = 0
                    except Exception:
                        rospy.logdebug(f"|> Failed to check {part} is visible")
                        continue

                    if part_mask.any():
                        rospy.logdebug(f"|> Person has {part} visible")
                    else:
                        rospy.logdebug(f"|> Person does not have {part} visible")
                        continue

                    if (
                        part.part_name == "torso_front"
                        or part.part_name == "torso_back"
                    ):
                        torso_mask = np.logical_or(torso_mask, part_mask)
                    elif (
                        part.part_name == "left_face" or part.part_name == "right_face"
                    ):
                        head_mask = np.logical_or(head_mask, part_mask)

                torso_mask_data, torso_mask_shape, torso_mask_dtype = numpy2message(
                    torso_mask
                )
                head_mask_data, head_mask_shape, head_mask_dtype = numpy2message(
                    head_mask
                )

                full_frame = cv2_img.cv2_img_to_msg(img)

                try:
                    rst = self.face_features(
                        full_frame,
                        head_mask_data,
                        head_mask_shape,
                        head_mask_dtype,
                        torso_mask_data,
                        torso_mask_shape,
                        torso_mask_dtype,
                    ).description
                except rospy.ServiceException as e:
                    rospy.logerr(f"Service call failed: {e}")
                    return "failed"

                people.append({"detection": person, "features": rst})

            # Userdata:
            # - people
            #   - - detection (YOLO)
            #     - parts
            #       - - part
            #         - mask
            userdata.people = people
            return "succeeded"
