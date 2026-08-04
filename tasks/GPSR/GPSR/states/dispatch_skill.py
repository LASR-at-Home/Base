from typing import Any

import rclpy
from rclpy.time import Time
import yasmin
from yasmin import Blackboard
import yasmin_ros

from std_msgs.msg import Header
from geometry_msgs.msg import Point, Pose, Quaternion, PointStamped
from rclpy.wait_for_message import wait_for_message
from sensor_msgs.msg import Image

from GPSR.world import load_locations
from GPSR.tts import say
from lasr_vision_interfaces.srv import (
    BodyPixKeypointDetection,
    DetectFaces as DetectFacesSrv,
)
from lasr_skills import (
    AskAndListen,
    DescribePeople,
    GoToLocation,
    HandoverObject,
    ReceiveObject,
    DetectWave,
    Rotate,
    FollowPerson,
    Wait,
    Detect3D,
    LookToPoint,
    Say,
    PlayMotion,
)

import time
from typing import List, Union, Optional


class DispatchSkill(yasmin.State):
    """YASMIN state that executes a skill chosen by the LLM."""

    def __init__(self, node):
        super().__init__(outcomes=["succeeded", "failed"])
        self.add_input_key("steps")
        self.node = node
        self.locations = load_locations(node)
        self._image_topic = "/head_front_camera/rgb/image_raw"
        self._detect_faces_client = self.node.create_client(  # Replace with ReId
            DetectFacesSrv, "/deepface/detect_faces"
        )
        self._bodypix_client = self.node.create_client(
            BodyPixKeypointDetection, "/bodypix/keypoint_detection"
        )

        self.task_bb = None

    # --- Support Methods
    def _first_arg(self, args: dict[str, Any], *names: str) -> str:
        for name in names:
            value = args.get(name)
            if isinstance(value, str) and value.strip():
                return value.strip()
        return ""

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

    # --- LASR SKILLS
    def _say(self, text):  # NEED Blackboard?
        if not text:
            return "succeeded"
        self.node.get_logger().info(f"Saying: {text}")
        say(self.node, text, self.task_bb)
        return "succeeded"

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
        return GoToLocation(location=pose)(self.task_bb)

    def _ask_and_listen(self, prompt: str) -> str:
        self.node.get_logger().info(f"Asking: {prompt}")
        try:
            outcome = AskAndListen(tts_phrase=prompt)(self.task_bb)
        except Exception as exc:
            self.node.get_logger().error(f"Ask/listen failed: {exc}")
            return "failed"
        return outcome

    def _detect_faces(
        self,
    ):  # TODO: Use Yolo to detect people then choose closest --- Update to find people faces   # Redundent as is replaced by REiD and spin breaks code.
        try:
            outcome = Detect3D(model="yolo11n.pt", filter=["person"])

            if outcome == "succeeded" and self.task_bb.get("detections_3d"):
                return True

        except Exception as exc:
            self.node.get_logger().error(
                f"Detect faces (placeholder with detect_wave) failed: {exc}"
            )
            return False

        return False

    def _detect_waving_person(self) -> bool:
        try:
            outcome = DetectWave()(self.task_bb)
            if outcome == "waving":
                return True

        except Exception as exc:
            self.node.get_logger().error(f"Ask/listen failed: {exc}")
            return False

        return False

    def _describe_person(self):
        try:
            outcome = DescribePeople()(self.task_bb)
        except Exception as exc:
            self.node.get_logger().error(f"Person description failed: {exc}")
            return None

        if outcome != "succeeded":
            self.node.get_logger().warning(
                f"Person description returned outcome '{outcome}'."
            )
            return None
        return self.task_bb.get("attributes")

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

    def _find_person(
        self, args: dict[str, Any]
    ):  # TODO:  Update with a 'search in area' which scans and rotates to find an object/person in the room
        location = self._first_arg(args, "location")
        name = self._first_arg(args, "name")
        gesture = self._first_arg(args, "gesture")
        clothes = self._first_arg(args, "clothes")

        # check around room

        if location:
            if self._go_to_location(location) == "failed":
                return "failed"

        if gesture:
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

        # TODO:  Ask for person to move infront of you. then wait
        self._say(
            "I cannot see you so can you please step infront of me. I will wait a few seconds"
        )
        time.sleep(3)

        detections = self._detect_faces()
        if detections:
            self._say("I found a person.")
            return "succeeded"

        self._say("I could not find a person.")
        return "failed"

    def _pick_up(self, args: dict[str, Any]):
        obj = self._first_arg(args, "object") or "object"
        try:
            outcome = ReceiveObject(object_name=obj)(self.task_bb)
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
            outcome = HandoverObject(object_name=obj)(self.task_bb)
        except Exception as exc:
            self.node.get_logger().error(f"HandoverObject failed: {exc}")
            self._say(f"Please take the {obj} from my hand.")
            return "failed"
        return outcome

    def _follow_person(self):
        try:
            outcome = FollowPerson()(self.task_bb)
        except Exception as exc:
            self.node.get_logger().error(f"FollowPerson failed: {exc}")
            self._say(f"I'm sorry. I am unable to follow you.")
            return "failed"
        return outcome

    def _find_object(self, object):
        def getPoint(blackboard):
            detections = blackboard.get("detections_3d").detected_objects
            if detections:
                blackboard["object_point"] = PointStamped(
                    header=Header(
                        frame_id="map",
                        stamp=Time().to_msg(),
                    ),
                    point=detections[0].point,
                )

                yasmin.YASMIN_LOG_INFO(f" An object is at {blackboard['object_point']}")
                return "succeeded"

            return "failed"

        try:
            yasmin.YASMIN_LOG_INFO(f"object to detect: {object}")
            filter_list = []
            if object is not None:
                filter_list = [object]

            sm = yasmin.StateMachine(outcomes=["succeeded", "failed"])
            sm.add_state(
                "LOOK_CENTRE",
                PlayMotion("look_centre"),
                transitions={
                    "succeeded": "DETECT3D_UP",
                    "aborted": "failed",
                    "canceled": "failed",
                },
            )
            sm.add_state(
                "DETECT3D_UP",
                Detect3D(filter=filter_list),
                transitions={"succeeded": "GET_POINT_UP", "failed": "failed"},
            )
            sm.add_state(
                "GET_POINT_UP",
                yasmin.CbState(outcomes=["succeeded", "failed"], callback=getPoint),
                transitions={"succeeded": "LOOK_AT_OBJECT", "failed": "LOOK_DOWN"},
            )
            sm.add_state(
                "LOOK_DOWN",
                PlayMotion("look_down_centre"),
                transitions={
                    "succeeded": "DETECT3D_DOWN",
                    "aborted": "failed",
                    "canceled": "failed",
                },
            )
            sm.add_state(
                "DETECT3D_DOWN",
                Detect3D(filter=filter_list),
                transitions={"succeeded": "GET_POINT_DOWN", "failed": "failed"},
            )
            sm.add_state(
                "GET_POINT_DOWN",
                yasmin.CbState(outcomes=["succeeded", "failed"], callback=getPoint),
                transitions={"succeeded": "LOOK_AT_OBJECT", "failed": "NO_OBJECT"},
            )

            sm.add_state(
                "LOOK_AT_OBJECT",
                LookToPoint(),
                transitions={
                    "succeeded": "CONFIRM",
                    "aborted": "failed",
                    "canceled": "failed",
                },
                remappings={"pointstamped": "object_point"},
            )
            sm.add_state(
                "CONFIRM",
                Say(text=f"I can see the {object}."),
                transitions={
                    "succeeded": "PRE_NAV",
                    "aborted": "PRE_NAV",
                    "canceled": "PRE_NAV",
                },
            )

            sm.add_state(
                "NO_OBJECT",
                Say(text=f"I cannot see the {object}."),
                transitions={
                    "succeeded": "PRE_NAV",
                    "aborted": "PRE_NAV",
                    "canceled": "PRE_NAV",
                },
            )
            sm.add_state(
                "PRE_NAV",
                PlayMotion("look_centre"),
                transitions={
                    "succeeded": "succeeded",
                    "aborted": "succeeded",
                    "canceled": "succeeded",
                },
            )
            outcome = sm(self.task_bb)
        except Exception as exc:
            self.node.get_logger().error(f"Find Object failed: {exc}")
            return "failed"
        return outcome

    def _execute_step(self, skill, args: Optional[Blackboard] = None):
        if skill == "say":  # CHECKED
            return self._say(args.get("text", ""))
        if skill == "go_to_location":  # CHECKED
            return self._go_to_location(args.get("location", ""))
        if skill == "guide_person":  # SOMEWHAT WORKS
            return self._guide_person(args)
        if (
            skill == "find_person"
        ):  # OUTDATED / BROKEN BACKEND - Disabled use of face detector
            return self._find_person(args)
        if skill == "get_person_info":  # SHOULD WORK
            return self._get_person_info(args)
        if skill == "pick_up":  # SHOULD WORK
            return self._pick_up(args)
        if skill == "place_object":  # SHOULD WORK
            return self._place_object(args)
        if skill == "give_to_person":  # SHOULD WORK
            return self._give_to_person(args)
        if skill == "follow_person":  # SHOULD WORK
            return self._follow_person()
        if skill == "find_object":  # SHOULD WORK
            return self._find_object(args.get("object", ""))

        # CAN ADD follow_person, find_object
        self.node.get_logger().info(f"Skipping skill '{skill}' (not yet actuated)")
        return "succeeded"

    def execute(self, blackboard):

        self.task_bb = yasmin.Blackboard()  # Create new shared Blackboard for the task
        for step in blackboard["steps"]:
            outcome = self._execute_step(step["skill"], step.get("args", {}))
            if outcome == "failed":
                self._execute_step(
                    skill="say",
                    args={
                        "text": "I couldn't complete that step. Moving to the next part of the plan."
                    },
                )
            time.sleep(0.5)
        return "succeeded"


from rclpy.node import Node
from threading import Thread
from rclpy.executors import MultiThreadedExecutor as Executor


class GPSRNode(Node):
    def __init__(self):
        super().__init__(
            node_name="gpsr",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )
        self._executor = Executor()
        self._executor.add_node(self)
        self._spin_thread = Thread(target=self._executor.spin)
        self._spin_thread.start()


def main():
    rclpy.init()
    node = GPSRNode()

    input_mode = node.get_parameter("input_mode").value.strip().lower()
    simulation = node.get_parameter("simulation").value
    node.get_logger().info(
        f"Starting GPSR state machine (input_mode={input_mode}, simulation={simulation})..."
    )

    try:
        bb = Blackboard()
        bb["steps"] = [
            # {"skill": "go_to_location", "args": {"location": "laundry"}},
            # {"skill": "go_to_location", "args": {"location": "shelf"}},
            {
                "skill": "find_object",
                "args": {"object": "red_bull", "location": "shelf"},
            },
            {"skill": "pick_up", "args": {"object": "red_bull"}},
            # {"skill": "go_to_location", "args": {"location": "living room"}},
            # {"skill": "go_to_location", "args": {"location": "coffee table"}},
            {"skill": "place_object", "args": {"location": "coffee table"}},
        ]

        outcome = DispatchSkill(node)(bb)

        yasmin.YASMIN_LOG_INFO(outcome)
    except Exception as e:
        yasmin.YASMIN_LOG_WARN(e)

    if rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
