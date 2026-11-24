from typing import Any

import rclpy
from rclpy.time import Time
import yasmin
from yasmin import Blackboard
import yasmin_ros

from std_msgs.msg import Header
from geometry_msgs.msg import Point, Pose, Quaternion, PointStamped
from sensor_msgs.msg import Image

from GPSR.world import load_locations, load_objects, normalise_name
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
from shapely.geometry import Polygon as ShapelyPolygon, Point as ShapelyPoint

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
        self._yolo_model = "yolo11n-seg.pt"
        self._detection_confidence = float(
            self._param("object_detection_confidence", 0.5)
        )
        # Detect3D subscribes to the cameras on construction and never unsubscribes,
        # so one instance per class filter is kept and reused (see _detect).
        self._detectors: dict[tuple, Detect3D] = {}

        self.task_bb = None

    # --- Support Methods
    def _param(self, name: str, default):
        if self.node.has_parameter(name):
            value = self.node.get_parameter(name).value
            if value is not None:
                return value
        return default

    def _first_arg(self, args: dict[str, Any], *names: str) -> str:
        for name in names:
            value = args.get(name)
            if isinstance(value, str) and value.strip():
                return value.strip()
        return ""

    ## --- CLAUDE start

    @staticmethod
    def _normalise(name: Any) -> str:
        return normalise_name(name)

    @staticmethod
    def _singular_forms(target: str) -> set:
        """`target` plus naive singular forms ("drinks" -> "drink", "dishes" -> "dish")."""
        forms = {target}
        if target.endswith("ies"):
            forms.add(target[:-3] + "y")
        if target.endswith("es"):
            forms.add(target[:-2])
        if target.endswith("s"):
            forms.add(target[:-1])
        return forms

    def _object_info(self, name: str) -> dict[str, Any]:
        """objects.yaml entry for a (normalised) object name or alias, {} if unknown."""
        if name in self.objects:
            return self.objects[name]
        for info in self.objects.values():
            if name in [self._normalise(a) for a in info.get("aliases") or []]:
                return info
        return {}

    def _detection_classes(self, obj_name: str) -> set:
        """Detector class names that count as the objects.yaml object `obj_name`.

        The object's own name, its aliases and its `detect_as` list (the YOLO classes
        the object is detected as, e.g. a coke can as "cup" / "bottle").
        """
        info = self.objects.get(obj_name, {})
        names = [obj_name, *(info.get("aliases") or []), *(info.get("detect_as") or [])]
        return {self._normalise(n) for n in names}

    def _target_objects(self, target: str) -> list:
        """objects.yaml names that `target` refers to - by name, alias or category."""
        forms = self._singular_forms(target)
        result = []
        for name, info in self.objects.items():
            aliases = {self._normalise(a) for a in info.get("aliases") or []}
            category = self._normalise(info.get("category"))
            if name in forms or aliases & forms or category in forms:
                result.append(name)
        return result

    def _object_label(self, detected_name: str, target: str = "") -> str:
        """Best objects.yaml name for a detection (restricted to `target` if given)."""
        detected = self._normalise(detected_name)
        candidates = self._target_objects(target) if target else list(self.objects)
        matching = [n for n in candidates if detected in self._detection_classes(n)]
        if not matching:
            return detected
        # Prefer the most specific object, e.g. "bottle" -> water bottle over coke
        return min(matching, key=lambda n: len(self.objects[n].get("detect_as") or []))

    def _matches_object(self, detected_name: str, target: str) -> bool:
        """Whether a detection counts as `target` (a normalised name, alias or category).

        The YOLO service silently detects *every* class when the filter contains a
        name its model does not know, so Detect3D is always run unfiltered and the
        detections are matched here instead: a detection matches if its class is the
        target itself, or a name / alias / `detect_as` class of any objects.yaml object
        the target refers to (by name, alias, or category - singular or plural).
        An empty target matches any object except people.
        """
        detected = self._normalise(detected_name)
        if not target:
            return detected != "person"
        if detected in self._singular_forms(target):
            return True
        return any(
            detected in self._detection_classes(name)
            for name in self._target_objects(target)
        )

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

    # Make the search polygon from the room or sub-location's four corners in locations.yaml.
    # Return None if there is no polygon.
    def _location_polygon(self, location_name):
        key = self._resolve_location(location_name)
        loc = self.locations.get(key) if key else None
        if not loc or "polygon" not in loc:
            return None
        p = loc["polygon"]
        return ShapelyPolygon(
            [p["top_left"], p["top_right"], p["bottom_right"], p["bottom_left"]]
        )

    @staticmethod
    def _in_polygon(detections, polygon):
        """Keep detections whose map (x, y) is inside `polygon`; no polygon = keep all."""
        if polygon is None:
            return detections
        return [
            d
            for d in detections
            if polygon.contains(ShapelyPoint(d.point.x, d.point.y))
        ]

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

    def _detect_gesture(self, gesture: str = "") -> bool:
        # TODO: update to get and use room polygon
        try:
            outcome = DetectGesture(gesture=gesture)(self.task_bb)
            if outcome == "succeeded" and len(self.task_bb["gesture_locations"]) > 0:
                return True

        except Exception as exc:
            self.node.get_logger().error(f"Detect Gestures failed: {exc}")
            return False

        return False

    def _describe_person(self, image: Optional[Image] = None):
        """Describe the person in `image` using the VLM service (/vlm/describe_people).

        Without an image, people are detected first (as in HRI: Detect3D writes
        `image_raw`, which GetVlmAttributes reads); the robot looks at the most
        confident person and the image is cropped to them.
        """
        if image is None:
            people = self._detect(["person"])
            if not people:
                self.node.get_logger().warning("No person detected to describe")
                return None
            person = max(people, key=lambda d: d.confidence)
            point = PointStamped(
                header=Header(frame_id="map", stamp=Time().to_msg()), point=person.point
            )
            self.node.get_logger().info(
                f"Person is at ({point.point.x:.2f}, {point.point.y:.2f}, {point.point.z:.2f})"
            )
            self._look_at(point)
            image = self._crop_image(self.task_bb["image_raw"], person.xywh)
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
                self._say(
                    f"Requested gesture {gesture_clean}, is not a handled gesture. Please wave instead."
                )
                gesture = "waving"
                gesture_clean = "waving"

            if self._detect_gesture(gesture):
                loc = self.task_bb["gesture_locations"][0]
                self.node.get_logger().info(
                    f"Person is at ({loc.point.x:.2f}, {loc.point.y:.2f}, {loc.point.z:.2f})"
                )
                self._look_at(loc)
                self._say(f"I found someone with gesture {gesture_clean}.")
                return "succeeded"

            # Backup Behaviour
            self._say(f"Please gesture {gesture_clean} so I can find you.")
            Wait(1)(self.task_bb)
            if self._detect_gesture(gesture):
                loc = self.task_bb["gesture_locations"][0]
                self.node.get_logger().info(
                    f"Person is at ({loc.point.x:.2f}, {loc.point.y:.2f}, {loc.point.z:.2f})"
                )
                self._look_at(loc)
                self._say(f"I found someone with gesture {gesture_clean}.")
                return "succeeded"

            self._say(f"I could not confirm someone with gesture {gesture_clean}.")

            return "failed"

        if name:
            self._say(f"{name}, please wave so I can find you.")
            if self._detect_gesture("waving"):
                loc = self.task_bb["gesture_locations"][0]
                self.node.get_logger().info(
                    f"Person is at ({loc.point.x:.2f}, {loc.point.y:.2f}, {loc.point.z:.2f})"
                )
                self._look_at(loc)
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
            person = self.task_bb["detections_3d"].detected_objects[0]
            point = PointStamped(
                header=Header(frame_id="map", stamp=Time().to_msg()), point=person.point
            )
            self.node.get_logger().info(
                f"Person is at ({point.point.x:.2f}, {point.point.y:.2f}, {point.point.z:.2f})"
            )
            self._look_at(point)
            self._say("I found a person.")
            return "succeeded"

        self._say("I could not find a person.")
        return "failed"

    def _pick_up(self, args: dict[str, Any]):
        obj = self._first_arg(args, "object") or "object"
        if "object_point" in self.task_bb.keys():
            point = self.task_bb["object_point"]
            self.node.get_logger().info(
                f"'{obj}' is at ({point.point.x:.2f}, {point.point.y:.2f}, {point.point.z:.2f})"
            )
            self._look_at(point)
        try:
            outcome = ReceiveObject(object_name=obj)(self.task_bb)
        except Exception as exc:
            self.node.get_logger().error(
                f"ReceiveObject for object pick-up failed: {exc}"
            )
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
        people = self._detect(["person"])
        if people:
            point = PointStamped(
                header=Header(frame_id="map", stamp=Time().to_msg()),
                point=people[0].point,
            )
            self.node.get_logger().info(
                f"Person is at ({point.point.x:.2f}, {point.point.y:.2f}, {point.point.z:.2f})"
            )
            self._look_at(point)
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
                    model=self._yolo_model,
                    filter=list(key),
                    confidence=self._detection_confidence,
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
        self, target: str = "", first_match: bool = False, polygon=None
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
            matches = self._in_polygon(
                [d for d in detections if self._matches_object(d.name, target)], polygon
            )
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

        matches = self._search_objects(target, polygon=self._location_polygon(location))
        self._play_motion("look_centre")
        if matches is None:
            self._say("I could not count the objects.")
            return "failed"

        count = len(matches)
        self.task_bb["count"] = count
        self.task_bb["object_detections"] = matches
        self._say(f"I counted {count} {target}.")
        return "succeeded"

    def _count_people(self, args: dict[str, Any]):
        location = self._first_arg(args, "location")
        gesture = self._first_arg(args, "gesture") or self._first_arg(args, "pose")
        gesture = gesture.lower().replace(" ", "_")
        clothes = self._normalise(self._first_arg(args, "clothes"))

        if location:
            if self._go_to_location(location) != "succeeded":
                return "failed"
        polygon = self._location_polygon(location)

        if gesture:
            if gesture not in VALID_GESTURES:
                self._say("I cannot recognise that gesture.")
                return "failed"
            try:
                outcome = DetectGesture(gesture=gesture)(self.task_bb)
            except Exception as exc:
                self.node.get_logger().error(f"Detect Gestures failed: {exc}")
                outcome = "failed"
            if outcome != "succeeded":
                self._say("I could not count the people.")
                return "failed"
            people = self._in_polygon(self.task_bb["gesture_locations"], polygon)
            suffix = " with gesture " + gesture.replace("_", " ")
        else:
            people = self._detect(["person"])
            if people is None:
                self._say("I could not count the people.")
                return "failed"
            people = self._in_polygon(people, polygon)
            suffix = ""
            if clothes:
                image = self.task_bb["image_raw"]
                people = [p for p in people if self._wears(image, p, clothes)]
                suffix = " wearing " + clothes

        count = len(people)
        self.task_bb["count"] = count
        word = "person" if count == 1 else "people"
        self._say(f"I counted {count} {word}{suffix}.")
        return "succeeded"

    def _wears(self, image, person, clothes):
        attrs = self._describe_person(self._crop_image(image, person.xywh))
        if not attrs:
            return False
        colour = str(attrs.get("shirt_color", "")).strip().lower()
        if colour in ("", "unknown"):
            return False
        return colour in clothes or clothes in colour

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
            label = self._object_label(detection.name, target)
            size = self.objects.get(label, {}).get("size")
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
        self._say(
            f"The {prop} {spoken} is the {self._object_label(chosen.name, target)}."
        )
        return "succeeded"

    def _execute_step(self, skill, args: Optional[dict] = None):
        args = args or {}
        if skill == "say":  # CHECKED
            text = args.get("text", "")
            count = (
                self.task_bb["count"] if "count" in self.task_bb.keys() else "unknown"
            )
            return self._say(text.replace("{count}", str(count)))
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
        if (
            skill == "give_to_person"
        ):  # WORKED IN ROBOCUP (RETEST) - WITHOUT MANIPULATION
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
                self.node.get_logger().warning(
                    f"Skill '{skill}' ended with '{outcome}'"
                )
                self._say(
                    "I couldn't complete that step. Moving to the next part of the plan."
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


def _load_test_cases() -> dict:
    """config/dispatch_tests.yaml -> {case name: [steps]}."""
    import yaml
    from GPSR.world import _pkg_config

    path = _pkg_config(None, "dispatch_tests.yaml")
    try:
        with open(path) as f:
            return (yaml.safe_load(f) or {}).get("cases") or {}
    except OSError:
        return {}


def main():
    """Run skill steps directly, without the LLM planner (see config/dispatch_tests.yaml)."""
    import json
    import os
    import sys

    from ament_index_python.packages import get_package_share_directory
    from rclpy.utilities import remove_ros_args

    args = remove_ros_args(sys.argv)[1:]
    cases = _load_test_cases()

    if args[:1] == ["--steps"] and len(args) == 2:
        steps = json.loads(args[1])
        label = "--steps"
    elif args and all(a in cases for a in args):
        steps = [step for name in args for step in cases[name]]
        label = " + ".join(args)
    else:
        unknown = [a for a in args if a not in cases]
        if unknown:
            print(f"Unknown test case(s): {unknown}")
        print("Usage: ros2 run GPSR dispatch_skill <case> [<case> ...]")
        print("       ros2 run GPSR dispatch_skill --steps '<json list of steps>'")
        print("Cases (config/dispatch_tests.yaml):")
        for name, case_steps in cases.items():
            print(f"  {name:16} " + ", ".join(s.get("skill", "?") for s in case_steps))
        return

    # Load params.yaml unless one was given, as the service does
    if "--params-file" not in sys.argv:
        sys.argv += [
            "--ros-args",
            "--params-file",
            os.path.join(get_package_share_directory("GPSR"), "config", "params.yaml"),
        ]

    rclpy.init()
    node = GPSRNode()
    yasmin_ros.set_ros_loggers(node)
    node.get_logger().info(f"Dispatching test '{label}':")
    for i, step in enumerate(steps, 1):
        node.get_logger().info(f"  [{i}] {step.get('skill')}({step.get('args', {})})")

    try:
        bb = Blackboard()
        bb["steps"] = steps
        outcome = DispatchSkill(node)(bb)
        yasmin.YASMIN_LOG_INFO(outcome)
    except Exception as e:
        yasmin.YASMIN_LOG_WARN(e)

    if rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
