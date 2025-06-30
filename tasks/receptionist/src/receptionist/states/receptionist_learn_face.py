"""The receptionist version of learn faces uses userdata for the name of the guest instead"""

import rospy
import smach

from lasr_vision_msgs.srv import AddFace, AddFaceRequest
from lasr_skills.vision import CropImage3D
from lasr_skills import Detect3D
from cv_bridge import CvBridge


class ReceptionistLearnFaces(smach.StateMachine):

    class LearnFaceState(smach.State):
        def __init__(self, guest_id: str):
            self._guest_id = guest_id
            self._bridge = CvBridge()
            self._learn_face = rospy.ServiceProxy("/lasr_vision_reid/add_face", AddFace)
            self._learn_face.wait_for_service()
            smach.State.__init__(
                self,
                outcomes=["succeeded", "failed"],
                input_keys=["cropped_images", "num_images"],
                output_keys=["num_images"],
            )

        def execute(self, userdata):
            try:
                request = AddFaceRequest(
                    image_raw=self._bridge.cv2_to_imgmsg(
                        userdata.cropped_images["person"], encoding="rgb8"
                    ),
                    name=self._guest_id,
                )
                response = self._learn_face(request)
                if response.success:
                    userdata.num_images += 1
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")
                return "failed"

            return "succeeded"

    class CheckDoneState(smach.State):
        def __init__(self, dataset_size: int):
            smach.State.__init__(
                self,
                outcomes=["succeeded", "failed"],
                input_keys=["num_images"],
            )
            self._dataset_size = dataset_size

        def execute(self, userdata):
            if userdata.num_images >= self._dataset_size:
                rospy.loginfo("Collected enough images for the guest.")
                return "succeeded"
            else:
                rospy.logwarn(
                    f"Not enough images collected for the guest: {userdata.num_images}/{self._dataset_size}."
                )
                return "failed"

    def __init__(self, guest_id: str, dataset_size: int = 10):
        smach.StateMachine.__init__(
            self, outcomes=["succeeded", "failed"], input_keys=["guest_data"]
        )
        self._guest_id = guest_id
        self._dataset_size = dataset_size

        with self:
            self.userdata.num_images = 0
            smach.StateMachine.add(
                "DETECT_3D",
                Detect3D(
                    filter=["person"],
                ),
                transitions={
                    "succeeded": "CROP_IMAGE_3D",
                    "failed": "failed",
                },
            )
            smach.StateMachine.add(
                "CROP_IMAGE_3D",
                CropImage3D(
                    filters=["person"],
                    crop_logic="nearest",
                    crop_type="masked",
                ),
                transitions={
                    "succeeded": "LEARN_FACE",
                    "failed": "failed",
                },
            )

            smach.StateMachine.add(
                "LEARN_FACE",
                self.LearnFaceState(self._guest_id),
                transitions={
                    "succeeded": "CHECK_DONE",
                    "failed": "failed",
                },
            )
            smach.StateMachine.add(
                "CHECK_DONE",
                self.CheckDoneState(self._dataset_size),
                transitions={
                    "succeeded": "succeeded",
                    "failed": "DETECT_3D",
                },
            )
