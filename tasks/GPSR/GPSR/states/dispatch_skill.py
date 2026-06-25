from typing import Any

import rclpy
import yasmin
from geometry_msgs.msg import Point, Pose, Quaternion
from rclpy.wait_for_message import wait_for_message
from sensor_msgs.msg import Image

from GPSR.world import load_locations
from GPSR.tts import say
from lasr_vision_interfaces.srv import BodyPixKeypointDetection, DetectFaces as DetectFacesSrv
from lasr_skills import AskAndListen, DescribePeople, GoToLocation, HandoverObject, ReceiveObject


class DispatchSkill(yasmin.State):
    """YASMIN state that executes a skill chosen by the LLM."""

    def __init__(self, node):
        super().__init__(outcomes=["succeeded", "failed"])
        self.add_input_key("steps")
        self.node = node
        self.locations = load_locations(node)
        self._image_topic = "/head_front_camera/rgb/image_raw"
        self._detect_faces_client = self.node.create_client(
            DetectFacesSrv, "/deepface/detect_faces"
        )
        self._bodypix_client = self.node.create_client(
            BodyPixKeypointDetection, "/bodypix/keypoint_detection"
        )

    def _say(self, text):
        if not text:
            return "succeeded"
        self.node.get_logger().info(f"Saying: {text}")
        say(self.node, text)
        return "succeeded"

    def _first_arg(self, args: dict[str, Any], *names: str) -> str:
        for name in names:
            value = args.get(name)
            if isinstance(value, str) and value.strip():
                return value.strip()
        return ""

    def _go_to_location(self, location_name):
        if location_name not in self.locations:
            self.node.get_logger().error(f"Unknown location: {location_name}")
            self._say(f"I don't know where {location_name} is")
            return "failed"
        loc = self.locations[location_name]
        pose = Pose(
            position=Point(
                x=float(loc["position"]["x"]),
                y=float(loc["position"]["y"]),
                z=float(loc["position"].get("z", 0.0)),
            ),
            orientation=Quaternion(
                x=float(loc["orientation"]["x"]),
                y=float(loc["orientation"]["y"]),
                z=float(loc["orientation"]["z"]),
                w=float(loc["orientation"]["w"]),
            ),
        )
        self.node.get_logger().info(f"Navigating to '{location_name}'")
        bb = yasmin.Blackboard()
        return GoToLocation(location=pose)(bb)

    def _ask_and_listen(self, prompt: str) -> str:
        self.node.get_logger().info(f"Asking: {prompt}")
        try:
            outcome = AskAndListen(tts_phrase=prompt)(yasmin.Blackboard())
        except Exception as exc:
            self.node.get_logger().error(f"Ask/listen failed: {exc}")
            return "failed"
        return outcome

    def _latest_image(self):
        try:
            return wait_for_message(
                node=self.node,
                topic=self._image_topic,
                msg_type=Image,
                timeout_sec=5.0,
            )
        except Exception as exc:
            self.node.get_logger().error(f"Failed to get camera image: {exc}")
            return None

    def _detect_faces(self):
        if not self._detect_faces_client.wait_for_service(timeout_sec=2.0):
            self.node.get_logger().warning("Face detection service is not available.")
            return []
        img_msg = self._latest_image()
        if img_msg is None:
            return []
        req = DetectFacesSrv.Request()
        req.image_raw = img_msg
        future = self._detect_faces_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        try:
            response = future.result()
        except Exception as exc:
            self.node.get_logger().error(f"Face detection failed: {exc}")
            return []
        return list(response.detections) if response else []

    def _detect_waving_person(self) -> bool:
        if not self._bodypix_client.wait_for_service(timeout_sec=2.0):
            self.node.get_logger().warning("BodyPix service is not available.")
            return False
        img_msg = self._latest_image()
        if img_msg is None:
            return False
        req = BodyPixKeypointDetection.Request()
        req.image_raw = img_msg
        req.dataset = ""
        req.confidence = 0.1
        req.keep_out_of_bounds = True
        future = self._bodypix_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        try:
            response = future.result()
        except Exception as exc:
            self.node.get_logger().error(f"Wave detection failed: {exc}")
            return False
        if not response:
            return False

        keypoints = {
            kp.keypoint_name: (float(kp.x), float(kp.y))
            for kp in response.normalized_keypoints
        }
        left = keypoints.get("leftShoulder"), keypoints.get("leftWrist")
        right = keypoints.get("rightShoulder"), keypoints.get("rightWrist")

        if left[0] and left[1] and left[1][1] < left[0][1]:
            return True
        if right[0] and right[1] and right[1][1] < right[0][1]:
            return True
        return False

    def _describe_person(self):
        try:
            bb = yasmin.Blackboard()
            outcome = DescribePeople()(bb)
        except Exception as exc:
            self.node.get_logger().error(f"Person description failed: {exc}")
            return None

        if outcome != "succeeded":
            self.node.get_logger().warning(
                f"Person description returned outcome '{outcome}'."
            )
            return None
        return bb.get("attributes")

    def _format_person_description(self, attributes: dict[str, Any] | None) -> str:
        if not attributes:
            return "I see a person."

        parts = []
        hair_color = str(attributes.get("hair_color", "")).strip()
        hair_length = str(attributes.get("hair_length", "")).strip()
        shirt_color = str(attributes.get("shirt_color", "")).strip()
        glasses = attributes.get("glasses")

        if hair_color and hair_color.lower() != "unknown":
            if hair_length and hair_length.lower() != "unknown":
                parts.append(f"{hair_color} {hair_length} hair")
            else:
                parts.append(f"{hair_color} hair")
        elif hair_length and hair_length.lower() != "unknown":
            parts.append(f"{hair_length} hair")

        if glasses:
            parts.append("glasses")
        if shirt_color and shirt_color.lower() != "unknown":
            parts.append(f"{shirt_color} shirt")

        if not parts:
            return "I see a person."
        return f"I see a person with {', '.join(parts)}."

    def _guide_person(self, args: dict[str, Any]):
        start = self._first_arg(args, "start")
        end = self._first_arg(args, "end", "destination", "location")
        name = self._first_arg(args, "name")

        if not end:
            self._say("I need a destination to guide a person.")
            return "failed"

        if start:
            if self._go_to_location(start) == "failed":
                return "failed"

        if name:
            self._say(f"{name}, please follow me to the {end}.")
        else:
            self._say(f"Please follow me to the {end}.")
        return self._go_to_location(end)

    def _find_person(self, args: dict[str, Any]):
        location = self._first_arg(args, "location")
        name = self._first_arg(args, "name")
        gesture = self._first_arg(args, "gesture")
        clothes = self._first_arg(args, "clothes")

        if location:
            if self._go_to_location(location) == "failed":
                return "failed"

        if gesture == "waving":
            if self._detect_waving_person():
                self._say("I found someone waving.")
                return "succeeded"
            self._say("Please wave so I can find you.")
            if self._detect_waving_person():
                self._say("I found someone waving.")
                return "succeeded"
            self._say("I could not confirm a waving person.")
            return "failed"

        if name:
            self._say(f"{name}, please wave so I can find you.")
            if self._detect_waving_person():
                self._say(f"I found someone who may be {name}.")
                return "succeeded"

        if clothes:
            attributes = self._describe_person()
            if attributes:
                shirt_color = str(attributes.get("shirt_color", "")).strip().lower()
                if shirt_color and clothes.lower() in shirt_color:
                    self._say(f"I found a person wearing a {clothes} shirt.")
                    return "succeeded"

        detections = self._detect_faces()
        if detections:
            if name:
                self._say(f"I found a person who may be {name}.")
            else:
                self._say("I found a person.")
            return "succeeded"

        self._say("I could not find a person.")
        return "failed"

    def _get_person_info(self, args: dict[str, Any]):
        location = self._first_arg(args, "location")
        if location:
            if self._go_to_location(location) == "failed":
                return "failed"
        attributes = self._describe_person()
        if attributes is None:
            self._say("I could not inspect the person right now.")
            return "failed"
        self._say(self._format_person_description(attributes))
        return "succeeded"

    def _pick_up(self, args: dict[str, Any]):
        obj = self._first_arg(args, "object") or "object"
        try:
            outcome = ReceiveObject(object_name=obj)(yasmin.Blackboard())
        except Exception as exc:
            self.node.get_logger().error(f"ReceiveObject failed: {exc}")
            return "failed"
        return outcome

    def _place_object(self, args: dict[str, Any]):
        obj = self._first_arg(args, "object") or "object"
        location = self._first_arg(args, "location", "surface", "target")
        if location:
            if self._go_to_location(location) == "failed":
                return "failed"
            prompt = f"Please place the {obj} on the {location}."
        else:
            prompt = f"Please place the {obj} where you want it."
        outcome = self._ask_and_listen(prompt + " Say done when finished.")
        return "succeeded" if outcome == "succeeded" else "failed"

    def _give_to_person(self, args: dict[str, Any]):
        obj = self._first_arg(args, "object") or "object"
        try:
            outcome = HandoverObject(node=self.node, object_name=obj, vertical=True).execute()
        except Exception as exc:
            self.node.get_logger().error(f"HandoverObject failed: {exc}")
            self._say(f"Please take the {obj} from my hand.")
            return "failed"
        return outcome

    def _execute_step(self, skill, args):
        if skill == "say":
            return self._say(args.get("text", ""))
        if skill == "go_to_location":
            return self._go_to_location(args.get("location", ""))
        if skill == "guide_person":
            return self._guide_person(args)
        if skill == "find_person":
            return self._find_person(args)
        if skill == "get_person_info":
            return self._get_person_info(args)
        if skill == "pick_up":
            return self._pick_up(args)
        if skill == "place_object":
            return self._place_object(args)
        if skill == "give_to_person":
            return self._give_to_person(args)
        self.node.get_logger().info(f"Skipping skill '{skill}' (not yet actuated)")
        return "succeeded"

    def execute(self, blackboard):
        for step in blackboard["steps"]:
            outcome = self._execute_step(step["skill"], step.get("args", {}))
            if outcome == "failed":
                return "failed"
        return "succeeded"
