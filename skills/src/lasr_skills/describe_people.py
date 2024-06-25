#!/usr/bin/env python3
import os
import cv2
import rospy
import smach
import cv2_img
import numpy as np

if "tiago" in os.environ["ROS_MASTER_URI"]:
    from lasr_skills import Say
from lasr_vision_msgs.srv import (
    YoloDetection,
    BodyPixMaskDetection,
    BodyPixMaskDetectionRequest,
    TorchFaceFeatureDetectionDescription,
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
            # conditional topic and crop method for flexibility
            rgb_topic = (
                "/xtion/rgb/image_raw"
                if "tiago" in os.environ["ROS_MASTER_URI"]
                else "/camera/image_raw"
            )
            crop_method = (
                "closest" if "tiago" in os.environ["ROS_MASTER_URI"] else "centered"
            )
            smach.StateMachine.add(
                "GET_IMAGE",
                GetCroppedImage(
                    object_name="person",
                    crop_method=crop_method,
                    rgb_topic=rgb_topic,
                    use_mask=False,
                ),
                transitions={
                    "succeeded": "CONVERT_IMAGE",
                    "failed": "SAY_GET_IMAGE_AGAIN",
                },
            )

            if "tiago" in os.environ["ROS_MASTER_URI"]:
                smach.StateMachine.add(
                    "SAY_GET_IMAGE_AGAIN",
                    Say(
                        text="Make sure you're looking into my eyes, I can't seem to see you."
                    ),
                    transitions={
                        "succeeded": "GET_IMAGE_AGAIN",
                        "preempted": "GET_IMAGE_AGAIN",
                        "aborted": "GET_IMAGE_AGAIN",
                    },
                )

            smach.StateMachine.add(
                "GET_IMAGE_AGAIN",
                GetCroppedImage(
                    object_name="person",
                    crop_method=crop_method,
                    rgb_topic=rgb_topic,
                    use_mask=False,
                ),
                transitions={"succeeded": "CONVERT_IMAGE", "failed": "SAY_CONTINUE"},
            )

            if "tiago" in os.environ["ROS_MASTER_URI"]:
                smach.StateMachine.add(
                    "SAY_CONTINUE",
                    Say(text="I can't see anyone, I will continue"),
                    transitions={
                        "succeeded": "failed",
                        "preempted": "failed",
                        "aborted": "failed",
                    },
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
                result = self.yolo(userdata.img_msg, "yolov8n-seg.pt", 0.5, 0.3)
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

                rst = self.face_features(
                    full_frame,
                    head_mask_data,
                    head_mask_shape,
                    head_mask_dtype,
                    torso_mask_data,
                    torso_mask_shape,
                    torso_mask_dtype,
                ).description

                people.append({"detection": person, "features": rst})

            # Userdata:
            # - people
            #   - - detection (YOLO)
            #     - parts
            #       - - part
            #         - mask
            userdata.people = people
            return "succeeded"
