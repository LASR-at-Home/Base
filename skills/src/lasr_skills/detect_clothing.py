import smach
import rclpy
from rclpy.node import Node
from typing import Optional
from lasr_vision_interfaces.srv import TorchFaceFeatureDetectionDescription
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

    def __init__(self,node:Node, clothing_to_detect: Optional[str] = None):
        """
        clothing_to_detect: "blue shirt"
        """
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["img_msg"],
        )
        self.node = node
        # self._clothing_to_detect = clothing_to_detect
        if clothing_to_detect is None or len(clothing_to_detect.split()) != 2:
            raise ValueError("Clothing to detect must be in format '<colour> <cloth>'")
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
                    self.node,
                    colour,
                    cloth,
                ),
                transitions={
                    "succeeded": "succeeded",
                    "failed": "failed",
                },
            )

    class Descriminator(smach.State):
        def __init__(
            self,
            node: Node,
            colour: str,
            cloth: str,
        ):
            smach.State.__init__(
                self,
                outcomes=[
                    "succeeded",
                    "failed",
                ],
                input_keys=[
                    "img_msg",
                ],
                output_keys=[],
            )
            self.node = node
            self.colour = colour
            self.cloth = cloth

            self.face_features = node.create_client(
                TorchFaceFeatureDetectionDescription,
                "/torch/detect/face_features"
            )

            while not self.face_features.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().info("Waiting for /torch/detect/face_features...")


        def execute(self, userdata):
            if self.colour not in colour_list or self.cloth not in cloth_list:
                return "failed"

            request = TorchFaceFeatureDetectionDescription.Request()
             
            request.image_raw = userdata.img_msg
            request.head_mask_data = []
            request.head_mask_shape = []
            request.head_mask_dtype = ""
            request.torso_mask_data = []
            request.torso_mask_shape = []
            request.torso_mask_dtype = ""

            future = self.face_features.call_async(request)
            rclpy.spin_until_future_complete(self.node, future)

            if not future.result():
                self.node.get_logger().error("Failed to get face features")
                return "failed"

            result = future.result()
            description = result.description.lower()

            if self.colour in description and self.cloth in description:
                return "succeeded"
            else:
                return "failed"

            # I removed the following part because we don't have numeric confidence scores and detailed feature breakdowns
            # I will remove it after testing 
            '''
            if len(result.people) == 0:
                return "failed"

            
            feature_dict = result.people[0]["features"]

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
            '''
