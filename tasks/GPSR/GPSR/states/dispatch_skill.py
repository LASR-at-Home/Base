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

from GPSR.world import load_locations, load_objects
from GPSR.tts import say
from lasr_skills import (
    AskAndListen,
    DescribePeople,
    GoToLocation,
    HandoverObject,
    ReceiveObject,
    DetectGesture,
    Rotate,
    FollowPerson,
    Wait,
    Detect3D,
    LookToPoint,
    Say,
    PlayMotion,
)
from lasr_skills.describe_people import GetVlmAttributes

import time
from typing import List, Union, Optional

# Gestures / poses understood by the gesture service (/gesture/detect)
VALID_GESTURES = [
    "lying_down",
    "pointing_left",
    "pointing_right",
    "raising_left_arm",
    "raising_right_arm",
    "sitting",
    "waving",
    "standing",
]

# Head poses used when searching a location for objects
SEARCH_MOTIONS = ["look_centre", "look_down_centre"]

OBJECT_PROPERTIES = ["biggest", "smallest", "heaviest", "lightest", "thinnest"]
SIZE_RANK = {"small": 0, "medium": 1, "large": 2}


class DispatchSkill(yasmin.State):
    """YASMIN state that executes a skill chosen by the LLM."""

    def __init__(self, node):
        super().__init__(outcomes=["succeeded", "failed"])
        self.add_input_key("steps")
        self.node = node
        self.locations = load_locations(node)
        # objects.yaml keyed by normalised name ("red_bull" -> "red bull")
        self.objects = {
            self._normalise(name): info or {}
            for name, info in load_objects(node).items()
        }
        self._image_topic = "/head_front_camera/rgb/image_raw"
        self._yolo_model = "yolo11n-seg.pt"
        # Detect3D subscribes to the cameras on construction and never unsubscribes,
        # so one instance per class filter is kept and reused (see _detect).
        self._detectors: dict[tuple, Detect3D] = {}

        self.task_bb = None

    # --- Support Methods
    def _first_arg(self, args: dict[str, Any], *names: str) -> str:
        for name in names:
            value = args.get(name)
            if isinstance(value, str) and value.strip():
                return value.strip()
        return ""

    ## --- CLAUDE start

    @staticmethod
    def _normalise(name: Any) -> str:
        return str(name or "").strip().lower().replace("_", " ")

    def _object_info(self, name: str) -> dict[str, Any]:
        """objects.yaml entry for a (normalised) object name or alias, {} if unknown."""
        if name in self.objects:
            return self.objects[name]
        for info in self.objects.values():
            if name in [self._normalise(a) for a in info.get("aliases") or []]:
                return info
        return {}

    def _matches_object(self, detected_name: str, target: str) -> bool:
        """Whether a detection counts as `target` (a normalised name, alias or category).

        The YOLO service silently detects *every* class when the filter contains a
        name its model does not know, so Detect3D is always run unfiltered and the
        detections are matched here instead: by class name, by objects.yaml alias,
        or by objects.yaml category (singular or plural, e.g. "drink" / "drinks").
        An empty target matches any object except people.
        """
        detected = self._normalise(detected_name)
        if not target:
            return detected != "person"
        forms = {target}
        if target.endswith("ies"):
            forms.add(target[:-3] + "y")
        if target.endswith("es"):
            forms.add(target[:-2])
        if target.endswith("s"):
            forms.add(target[:-1])
        if detected in forms:
            return True
        info = self._object_info(detected)
        if not info:
            return False
        if any(info is self._object_info(form) for form in forms):
            return True
        return self._normalise(info.get("category")) in forms

    def _crop_image(self, image: Image, xywh) -> Image:
        """Crop an 8-bit sensor_msgs/Image to a pixel bounding box."""
        bytes_per_pixel = image.step // image.width
        x = max(0, min(int(xywh[0]), image.width - 1))
        y = max(0, min(int(xywh[1]), image.height - 1))
        w = max(1, min(int(xywh[2]), image.width - x))
        h = max(1, min(int(xywh[3]), image.height - y))
        data = bytes(image.data)
        rows = [
            data[
                row * image.step
                + x * bytes_per_pixel : row * image.step
                + (x + w) * bytes_per_pixel
            ]
            for row in range(y, y + h)
        ]
        return Image(
            header=image.header,
            height=h,
            width=w,
            encoding=image.encoding,
            is_bigendian=image.is_bigendian,
            step=w * bytes_per_pixel,
            data=b"".join(rows),
        )

    ## --- CLAUDE end
    
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

    def _resolve_location(self, location_name: str) -> Optional[str]:
        """Match a planner-supplied name to a key in locations.yaml (case / _ / space insensitive)."""
        if location_name in self.locations:
            return location_name
        norm = str(location_name).strip().lower().replace("_", " ")
        for key in self.locations:
            if key.strip().lower().replace("_", " ") == norm:
                return key
        return None

    def _go_to_location(self, location_name):
        key = self._resolve_location(location_name)
        if key is None:
            self.node.get_logger().error(f"Unknown location: {location_name}")
            self._say(f"I don't know where {location_name} is")
            return "failed"
        loc = self.locations[key]
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

    def _detect_faces(self) -> bool:
        """Return True if at least one person is visible in front of the camera."""
        # TODO: Update to use ReID / face detection rather than whole-body person
        try:
            outcome = Detect3D(model="yolo11n-seg.pt", filter=["person"])(self.task_bb)
            if outcome != "succeeded":
                return False
            if "detections_3d" not in self.task_bb.keys():
                return False
            return bool(self.task_bb["detections_3d"].detected_objects)
        except Exception as exc:
            self.node.get_logger().error(
                f"Detect faces (Detect3d with person filter) failed: {exc}"
            )
            return False

    def _detect_gesture(self, gesture: str="") -> bool:
        #TODO: update to get and use room polygon
        try:
            outcome = DetectGesture(gesture=gesture)(self.task_bb)
            if outcome == "succeeded" and len(self.task_bb["gesture_locations"]) > 0:
                return True

        except Exception as exc:
            self.node.get_logger().error(f"Detect Gestures failed: {exc}")
            return False

        return False

    def _describe_person(self, image: Optional[Image] = None):
        """Describe the person in `image` (default: the current camera image) using the VLM service.

        DescribePeople is not used as its GetImage state writes `img_msg` while the
        VLM state reads `image_raw`, so the image is fetched here and passed directly.
        """
        if image is None:
            image = self._latest_image()
        if image is None:
            return None
        self.task_bb["image_raw"] = image
        try:
            outcome = GetVlmAttributes()(self.task_bb)
        except Exception as exc:
            self.node.get_logger().error(f"Person description failed: {exc}")
            return None

        if outcome != "succeeded":
            self.node.get_logger().warning(
                f"Person description returned outcome '{outcome}'."
            )
            return None
        return self.task_bb["attributes"]

    def _guide_person(self, args: dict[str, Any]):
        start = self._first_arg(args, "start")
        end = self._first_arg(args, "end", "destination", "location")
        name = self._first_arg(args, "name")

        if not end:
            self._say("I need a destination to guide a person.")
            return "failed"

        # if a start location was defined ensures the robot is there
        if start:
            if self._go_to_location(start) == "failed":
                return "failed"

        if name:
            self._say(f"{name}, please follow me to the {end}.")
        else:
            self._say(f"Please follow me to the {end}.")
        return self._go_to_location(end)

    def _find_person(self, args: dict[str, Any]):  
        # TODO:  Update with a 'search in area' which scans and rotates to find an object/person in the room - creatrea a new service/skill for this
        location = self._first_arg(args, "location")
        name = self._first_arg(args, "name")
        gesture = self._first_arg(args, "gesture")
        clothes = self._first_arg(args, "clothes")

        if location:
            if self._go_to_location(location) == "failed":
                return "failed"

        if gesture:
            gesture_clean = gesture.replace("_", " ")

            # If gesture is not handled default to waving
            if gesture not in VALID_GESTURES:
                self._say(f"Requested gesture {gesture_clean}, is not a handled gesture. Please wave instead.")
                gesture = "waving"
                gesture_clean = "waving"


            if self._detect_gesture(gesture):
                self._say(f"I found someone with gesture {gesture_clean}.")
                return "succeeded"
            
            # Backup Behaviour
            self._say(f"Please gesture {gesture_clean} so I can find you.")
            Wait(1)(self.task_bb)
            if self._detect_gesture(gesture):
                self._say(f"I found someone with gesture {gesture_clean}.")
                return "succeeded"
            
            self._say(f"I could not confirm someone with gesture {gesture_clean}.")
            
            return "failed"
            

        if name:
            self._say(f"{name}, please wave so I can find you.")
            if self._detect_gesture("waving"):
                self._say(f"I found someone who may be {name}.")
                return "succeeded"

        if clothes:
            attributes = self._describe_person()
            if attributes:
                shirt_color = str(attributes.get("shirt_color", "")).strip().lower()
                if shirt_color and clothes.lower() in shirt_color:
                    self._say(f"I found a person wearing a {clothes} shirt.")
                    return "succeeded"

        self._say(
            "I cannot see you so can you please step infront of me. I will wait a few seconds"
        )
        Wait(3)(self.task_bb)

        # Final fallback - Just detect anybody 
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
            self.node.get_logger().error(f"ReceiveObject for object pick-up failed: {exc}")
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

    ## --- CLAUDE start

    # --- Object / people perception
    # States are called directly (not wired into a StateMachine) so that every outcome
    # (succeeded / aborted / canceled / timeout) is handled by a single comparison.
    def _play_motion(self, motion_name: str) -> bool:
        try:
            return PlayMotion(motion_name)(self.task_bb) == "succeeded"
        except Exception as exc:
            self.node.get_logger().error(f"PlayMotion '{motion_name}' failed: {exc}")
            return False

    def _look_at(self, point: PointStamped) -> bool:
        try:
            return LookToPoint(pointstamped=point)(self.task_bb) == "succeeded"
        except Exception as exc:
            self.node.get_logger().error(f"LookToPoint failed: {exc}")
            return False

    def _detect(self, class_filter: Optional[List[str]] = None) -> Optional[list]:
        """Run Detect3D (map frame); return its detections, or None if detection failed.

        The detections are also left in task_bb['detections_3d'] and the image they
        were made on in task_bb['image_raw'].
        """
        key = tuple(class_filter or [])
        try:
            if key not in self._detectors:
                self._detectors[key] = Detect3D(
                    model=self._yolo_model, filter=list(key)
                )
            outcome = self._detectors[key](self.task_bb)
        except Exception as exc:
            self.node.get_logger().error(f"Detect3D failed: {exc}")
            return None
        if outcome != "succeeded":
            self.node.get_logger().error(f"Detect3D ended with '{outcome}'")
            return None
        return list(self.task_bb["detections_3d"].detected_objects)

    def _search_objects(
        self, target: str = "", first_match: bool = False
    ) -> Optional[list]:
        """Look centre, then down, detecting objects at each head pose.

        Returns the detections matching `target` (see _matches_object) from the head
        pose that saw the most of them - a single view, so objects visible in both
        poses are not counted twice - or None if detection failed at every pose.
        With `first_match` the search stops at the first pose with a match.
        The head is left at the last pose used; callers return it to look_centre.
        """
        best = None
        for motion in SEARCH_MOTIONS:
            if not self._play_motion(motion):
                self.node.get_logger().warning(
                    f"Head motion '{motion}' failed, detecting from the current head pose"
                )
            detections = self._detect()
            if detections is None:
                continue
            matches = [d for d in detections if self._matches_object(d.name, target)]
            if best is None or len(matches) > len(best):
                best = matches
            if first_match and matches:
                break
        return best

    def _record_found_object(self, detection, matches: list) -> None:
        """Store a found object on the task blackboard and look at it."""
        point = PointStamped(
            # Zero stamp: use the latest available transform when pointing the head
            header=Header(frame_id="map", stamp=Time().to_msg()),
            point=detection.point,
        )
        self.task_bb["object_point"] = point
        self.task_bb["found_object"] = detection.name
        self.task_bb["object_detections"] = matches
        self.node.get_logger().info(
            f"'{detection.name}' is at ({point.point.x:.2f}, {point.point.y:.2f}, {point.point.z:.2f})"
        )
        if not self._look_at(point):
            self.node.get_logger().warning(f"Could not look at '{detection.name}'")

    def _find_object(self, args: dict[str, Any]):
        location = self._first_arg(args, "location")
        target = self._normalise(self._first_arg(args, "object"))
        spoken = target or "object"

        if location:
            if self._go_to_location(location) != "succeeded":
                return "failed"

        matches = self._search_objects(target, first_match=True)
        if matches:
            best = max(matches, key=lambda d: d.confidence)
            self._record_found_object(best, matches)
            self._say(f"I can see the {spoken}.")
        else:
            self._say(f"I cannot see the {spoken}.")
        self._play_motion("look_centre")
        return "succeeded" if matches else "failed"

    def _count_objects(self, args: dict[str, Any]):
        location = self._first_arg(args, "location")
        target = self._normalise(self._first_arg(args, "object"))

        if location:
            if self._go_to_location(location) != "succeeded":
                return "failed"

        matches = self._search_objects(target)
        self._play_motion("look_centre")
        if matches is None:
            self._say("I could not count the objects.")
            return "failed"

        count = len(matches)
        self.task_bb["count"] = count
        self.task_bb["object_detections"] = matches
        label = f"{target} item" if target else "item"
        self._say(f"I counted {count} {label}{'' if count == 1 else 's'}.")
        return "succeeded"

    def _count_people(self, args: dict[str, Any]):
        location = self._first_arg(args, "location")
        gesture = self._first_arg(args, "gesture").lower().replace(" ", "_")
        pose = self._first_arg(args, "pose").lower().replace(" ", "_")
        clothes = self._normalise(self._first_arg(args, "clothes"))

        if location:
            if self._go_to_location(location) != "succeeded":
                return "failed"

        # The gesture service checks one gesture per call and its locations cannot be
        # matched to VLM descriptions, so only one filter is applied:
        # gesture, else pose, else clothes.
        requested = gesture or pose
        ignored = [f for f in (gesture and pose, requested and clothes) if f]
        if ignored:
            self.node.get_logger().warning(
                f"count_people applies one filter at a time, ignoring: {ignored}"
            )

        if requested:
            # Poses (sitting / standing / lying_down) are gestures of the gesture service
            requested_clean = requested.replace("_", " ")
            if requested not in VALID_GESTURES:
                self._say(f"I cannot recognise people who are {requested_clean}.")
                return "failed"
            try:
                outcome = DetectGesture(gesture=requested)(self.task_bb)
            except Exception as exc:
                self.node.get_logger().error(f"Detect Gestures failed: {exc}")
                outcome = "failed"
            if outcome != "succeeded":
                self._say("I could not count the people.")
                return "failed"
            count = len(self.task_bb["gesture_locations"])
            suffix = f" with gesture {requested_clean}"
        else:
            people = self._detect(["person"])
            if people is None:
                self._say("I could not count the people.")
                return "failed"
            if clothes:
                # Best effort: each detected person is cropped out of the detection
                # image and described by the VLM, which only reports a shirt colour,
                # so `clothes` can only be matched on the colour of the top.
                image = self.task_bb["image_raw"]
                count, described = 0, 0
                for person in people:
                    attributes = self._describe_person(
                        self._crop_image(image, person.xywh)
                    )
                    if attributes is None:
                        continue
                    described += 1
                    shirt_color = str(attributes.get("shirt_color", "")).strip().lower()
                    if shirt_color and shirt_color != "unknown":
                        if shirt_color in clothes or clothes in shirt_color:
                            count += 1
                if people and not described:
                    self._say("I could not check what the people are wearing.")
                    return "failed"
                suffix = f" wearing {clothes}"
            else:
                count = len(people)
                suffix = ""

        self.task_bb["count"] = count
        self._say(f"I counted {count} {'person' if count == 1 else 'people'}{suffix}.")
        return "succeeded"

    def _find_object_by_property(self, args: dict[str, Any]):
        location = self._first_arg(args, "location")
        prop = self._first_arg(args, "property").lower()
        target = self._normalise(self._first_arg(args, "object"))
        spoken = target or "object"

        if prop not in OBJECT_PROPERTIES:
            self._say(f"I do not know how to find the {prop or 'requested'} {spoken}.")
            return "failed"

        if location:
            if self._go_to_location(location) != "succeeded":
                return "failed"

        matches = self._search_objects(target)
        if not matches:
            self._say(f"I cannot see any {spoken}.")
            self._play_motion("look_centre")
            return "failed"

        # Detection3D carries no 3D size, only a pixel bounding box (xywh) and a
        # point, so sizes are compared in pixels. All candidates come from a single
        # image of the same surface, so they are at a similar distance.
        def area(detection) -> int:
            return detection.xywh[2] * detection.xywh[3]

        def thickness(detection) -> int:
            return min(detection.xywh[2], detection.xywh[3])

        # Weight proxy: the objects.yaml `size` class (unknown objects count as
        # medium), with the pixel area as a tie-break.
        def weight(detection) -> tuple:
            size = self._object_info(self._normalise(detection.name)).get("size")
            return (SIZE_RANK.get(size, SIZE_RANK["medium"]), area(detection))

        if prop == "biggest":
            chosen = max(matches, key=area)
        elif prop == "smallest":
            chosen = min(matches, key=area)
        elif prop == "thinnest":
            chosen = min(matches, key=thickness)
        elif prop == "heaviest":
            chosen = max(matches, key=weight)
        else:
            chosen = min(matches, key=weight)

        self._record_found_object(chosen, matches)
        self._say(f"The {prop} {spoken} is the {self._normalise(chosen.name)}.")
        self._play_motion("look_centre")
        return "succeeded"

    def _execute_step(self, skill, args: Optional[dict] = None):
        args = args or {}
        if skill == "say":  # CHECKED
            return self._say(args.get("text", ""))
        if skill == "go_to_location":  # CHECKED
            return self._go_to_location(args.get("location", ""))
        if skill == "guide_person":  # SHOULD WORK (only uses go_to_location)
            return self._guide_person(args)
        if skill == "find_person":
            return self._find_person(args)
        if skill == "get_person_info":  # UNTESTED
            return self._get_person_info(args)
        if skill == "pick_up":  # WORKED IN ROBOCUP (RETEST) - WITHOUT MANIPULATION
            return self._pick_up(args)
        if skill == "place_object":  # WORKED IN ROBOCUP (RETEST) - WITHOUT MANIPULATION
            return self._place_object(args)
        if skill == "give_to_person":  # WORKED IN ROBOCUP (RETEST) - WITHOUT MANIPULATION
            return self._give_to_person(args)
        if skill == "follow_person":  # UNTESTED
            return self._follow_person()
        if skill == "find_object":  # UNTESTED
            return self._find_object(args)
        if skill == "count_objects":  # UNTESTED
            return self._count_objects(args)
        if skill == "count_people":  # UNTESTED
            return self._count_people(args)
        if skill == "find_object_by_property":  # UNTESTED
            return self._find_object_by_property(args)
        if skill == "answer_question":
            # The planner already adds a `say` step containing the answer, so nothing to do.
            return "succeeded"

        self.node.get_logger().error(f"Skill '{skill}' is not implemented")
        return "failed"

    def execute(self, blackboard):

        self.task_bb = yasmin.Blackboard()  # Create new shared Blackboard for the task
        for step in blackboard["steps"]:
            skill, args = step.get("skill", ""), step.get("args", {})
            try:
                outcome = self._execute_step(skill, args)
            except Exception as exc:
                self.node.get_logger().error(f"Skill '{skill}' raised: {exc}")
                outcome = "failed"
            if outcome != "succeeded":
                self.node.get_logger().warning(f"Skill '{skill}' ended with '{outcome}'")
                self._say("I couldn't complete that step. Moving to the next part of the plan.")
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
    yasmin_ros.set_ros_loggers(node)

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
            # {"skill": "count_objects", "args": {"object": "drink", "location": "cabinet"}},
            # {"skill": "count_people", "args": {"location": "living room"}},
            # {"skill": "count_people", "args": {"pose": "sitting", "location": "living room"}},
            # {"skill": "count_people", "args": {"gesture": "waving", "location": "kitchen"}},
            # {"skill": "count_people", "args": {"clothes": "red", "location": "kitchen"}},
            # {
            #     "skill": "find_object_by_property",
            #     "args": {"property": "biggest", "object": "fruit", "location": "dinner table"},
            # },
            # {"skill": "answer_question", "args": {}},
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
