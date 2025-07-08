import smach
import smach_ros
from shapely.geometry.polygon import Polygon
from typing import List, Literal, Optional
from lasr_skills import Detect3DInArea
from lasr_skills.vision import CropImage3D


class ObjectComparison(smach.StateMachine):

    _object_comp_list: List[str] = [
        "biggest",
        "largets",
        "smallest",
        "heaviest",
        "lightest",
        "thinnest",
    ]

    # TODO: fill this in

    _smallest_list: List[str] = [
        "dishwasher_tab",
        "candle",
        "strawberry",
        "tictac",
        "plum",
        "lemon",
        "pear",
        "peach",
        "orange",
        "apple",
        "sponges",
        "fork",
        "knife",
        "spoon",
        "banana",
        "dubbelfris",
        "cola",
        "ice_tea",
        "fanta",
        "milk",
        "pea_soup",
        "sausages",
        "cup",
        "stroopwafel",
        "soap",
        "curry",
        "water",
        "crisps",
        "liquorice",
        "candy",
        "hagelslag",
        "pancake_mix",
        "mayonaise",
        "bowl",
        "plate",
        "washcloth",
        "pringles",
        "big_coke",
    ]
    _biggest_list: List[str] = reversed(_smallest_list)

    _largest_list: List[str] = _biggest_list

    _heaviest_list: List[str] = [
        "big_coke",
        "pea_soup",
        "milk",
        "mayonaise",
        "cola",
        "ice_tea",
        "fanta",
        "dubbelfris",
        "water",
        "pancake_mix",
        "curry",
        "soap",
        "sausages",
        "apple",
        "orange",
        "peach",
        "pear",
        "lemon",
        "plum",
        "banana",
        "stroopwafel",
        "cup",
        "bowl",
        "plate",
        "washcloth",
        "sponges",
        "pringles",
        "crisps",
        "hagelslag",
        "liquorice",
        "candy",
        "knife",
        "fork",
        "spoon",
        "candle",
        "strawberry",
        "tictac",
        "dishwasher_tab",
    ]

    _lightest_list: List[str] = reversed(_heaviest_list)

    _thinnest_list: List[str] = [
        "knife",
        "fork",
        "spoon",
        "candle",
        "strawberry",
        "tictac",
        "dishwasher_tab",
        "liquorice",
        "candy",
        "crisps",
        "stroopwafel",
        "soap",
        "sponges",
        "washcloth",
        "plate",
        "bowl",
        "cup",
        "pancake_mix",
        "hagelslag",
        "curry",
        "mayonaise",
        "pea_soup",
        "sausages",
        "milk",
        "water",
        "dubbelfris",
        "cola",
        "ice_tea",
        "fanta",
        "big_coke",
        "apple",
        "orange",
        "peach",
        "pear",
        "lemon",
        "plum",
        "banana",
        "pringles",
    ]

    _query: Literal[
        "biggest", "largest", "smallest", "heaviest", "lightest", "thinnest"
    ]

    def _compare_objects(self, userdata):
        # Cropped detections are a dictionary with keys as object names and values as lists of dictionaries
        # Each dictionary contains "detection_3d" and "cropped_image"
        # Example: {"apple": [{"detection_3d": Detection3D, "cropped_image": Image}, ...], ...}
        detections = userdata.cropped_detections
        detected_objects = set(detections.keys())

        if self._query == "biggest":
            biggest_object = next(
                (obj for obj in self._biggest_list if obj in detected_objects), None
            )
            if not biggest_object:
                return "failed"
            userdata.query_result = biggest_object
            return "succeeded"
        elif self._query == "largest":
            largest_object = next(
                (obj for obj in self._largest_list if obj in detected_objects), None
            )
            if not largest_object:
                return "failed"
            userdata.query_result = largest_object
            return "succeeded"
        elif self._query == "smallest":
            smallest_object = next(
                (obj for obj in self._smallest_list if obj in detected_objects), None
            )
            if not smallest_object:
                return "failed"
            userdata.query_result = smallest_object
            return "succeeded"
        elif self._query == "heaviest":
            heaviest_object = next(
                (obj for obj in self._heaviest_list if obj in detected_objects), None
            )
            if not heaviest_object:
                return "failed"
            userdata.query_result = heaviest_object
            return "succeeded"
        elif self._query == "lightest":
            lightest_object = next(
                (obj for obj in self._lightest_list if obj in detected_objects), None
            )
            if not lightest_object:
                return "failed"
            userdata.query_result = lightest_object
            return "succeeded"
        elif self._query == "thinnest":
            thinnest_object = next(
                (obj for obj in self._thinnest_list if obj in detected_objects), None
            )
            if not thinnest_object:
                return "failed"
            userdata.query_result = thinnest_object
            return "succeeded"
        else:
            return "failed"

    def __init__(
        self,
        query: Literal[
            "biggest", "largest", "smallest", "heaviest", "lightest", "thinnest"
        ],
        area_polygon: Polygon,  # input key
        model: str = "best.pt",
        objects: Optional[List[str]] = None,
        confidence: float = 0.5,
        nms: float = 0.3,
    ):

        self._query = query

        smach.StateMachine.__init__(
            self, outcomes=["succeeded", "failed"], output_keys=["query_result"]
        )

        with self:
            smach.StateMachine.add(
                "DETECT_OBJECTS",
                Detect3DInArea(
                    area_polygon=area_polygon,  # Contains the whole room, so need to crop
                    filter=objects,
                    model=model,
                    confidence=confidence,
                    nms=nms,
                    target_frame="map",
                ),
                transitions={
                    "succeeded": "CROP_OBJECTS",
                    "failed": "failed",
                },
                remapping={
                    "detections_3d": "detections_3d",
                    "image_raw": "image_raw",
                },
            )
            smach.StateMachine.add(
                "CROP_OBJECTS",
                CropImage3D(
                    robot_pose_topic="/robot_pose",
                    filters=objects,
                    crop_logic="nearest",
                    crop_types="masked",
                ),
                transitions={
                    "succeeded": "COMPARE_OBJECTS",
                    "failed": "failed",
                },
                remapping={
                    "cropped_detections": "cropped_detections"  # Dict[List[Dict[str, Union[Image, Detection3D]]]]
                },
            )

            smach.StateMachine.add(
                "COMPARE_OBJECTS",
                smach.CBState(
                    self._compare_objects,
                    outcomes=["succeeded", "failed"],
                    input_keys=["cropped_detections"],
                ),
                transitions={"succeeded": "succeeded"},
            )
