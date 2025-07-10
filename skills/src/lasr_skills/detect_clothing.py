import smach
import rospy

from typing import Optional
from lasr_vision_msgs.srv import (
    TorchFaceFeatureDetectionDescription,
)
from lasr_skills import DescribePeople


colour_list = ["blue", "yellow", "black", "white", "red", "orange", "gray"]
cloth_list = ["t shirt", "shirt", "blouse", "sweater", "coat", "jacket"]
cloth_type_map = {
    "t shirt": "short sleeve top",
    "shirt": "long sleeve top",
    "blouse": "long sleeve top",
    "sweater": "long sleeve top",
    "coat": "long sleeve outwear",
    "jacket": "long sleeve outwear",
}
cloth_type_rough_map = {
    "short sleeve top": "top",
    "long sleeve top": "top",
    "long sleeve outwear": "outwear",
}


class DetectClothing(smach.StateMachine):
    def __init__(self, clothing_to_detect: Optional[str] = None):
        """
        clothing_to_detect: "blue shirt"
        """
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["img_msg"],
        )

        # self._clothing_to_detect = clothing_to_detect

        colour = clothing_to_detect.split[0]
        cloth = clothing_to_detect.split[1]

        with self:
            smach.StateMachine.add(
                "GET_ATTRIBUTES",
                DescribePeople(),
                transitions={
                    "succeeded": "DECIDE",
                    "failed": "failed",
                },
            )

            smach.StateMachine.add(
                "DECIDE",
                self.Descriminator(
                    colour=colour,
                    cloth=cloth,
                ),
                transitions={
                    "succeeded": "succeeded",
                    "failed": "failed",
                },
            )

    class Descriminator(smach.State):
        def __init__(
            self,
            colour: str,
            cloth: str,
        ):
            smach.State.__init__(
                self,
                outcomes=[
                    "succeeded",
                    "fail",
                ],
                input_keys=[
                    "img_msg",
                ],
                output_keys=[],
            )
            self.colour = colour
            self.cloth = cloth

            self.face_features = rospy.ServiceProxy(
                "/torch/detect/face_features", TorchFaceFeatureDetectionDescription
            )

            def execute(self, userdata):
                if self.colour not in colour_list or self.cloth not in cloth_list:
                    return "failed"
                if len(userdata.people) == 0:
                    return "failed"

                feature_dict = userdata.people[0]["features"]

                inquired_cloth_type = cloth_type_map[self.cloth]

                inquired_rough_cloth_type = cloth_type_rough_map[inquired_cloth_type]

                confidence = feature_dict[inquired_rough_cloth_type]

                if confidence < 0.25:
                    return "failed"

                cloth_type = cloth_type_rough_map["max_" + inquired_rough_cloth_type]

                if inquired_cloth_type != cloth_type:
                    return "failed"

                colour_percentage = feature_dict[inquired_cloth_type + " colour"]

                largest_colour = max(colour_percentage, key=colour_percentage.get)

                if inquired_cloth_type == largest_colour:
                    return "succeeded"

                return "failed"
