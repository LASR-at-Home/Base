#!/usr/bin/env python3
import smach
from lasr_skills import Detect3DInArea
from lasr_skills import Detect3D
from lasr_skills import Detect
from shapely.geometry.polygon import Polygon
from typing import List, Union, Dict


class ObjectComparison(smach.StateMachine):
    class CountObjectTypes(smach.State):
        def __init__(self, area_polygon: Polygon):
            smach.State.__init__(
                self,
                outcomes=["succeeded", "failed"],
                input_keys=["detections_3d"],
                output_keys=["detections_types", "object_dict"],
            )

        def count_types(self, detections):
            """
            Count the number of different object types in detections.

            :param detections: List of detection tuples (object_type, position)
            :return: Dictionary with object types as keys and counts as values
            """
            object_counts = {}
            for detection in detections:
                object_type = detection.name

                if len(object_counts.keys()) == 0:
                    object_counts[object_type] = 1
                else:
                    if object_type in object_counts.keys():
                        object_counts[object_type] += 1
                    else:
                        object_counts[object_type] = 1
            return object_counts

        def execute(self, userdata):
            print(userdata.detections_3d)
            filtered_detections = userdata.detections_3d
            rospy.loginfo(filtered_detections)
            rospy.loginfo(type(filtered_detections.detected_objects[0]))
            object_counts = self.count_types(filtered_detections.detected_objects)
            print(object_counts)
            userdata.object_dict = object_counts
            userdata.detections_types = object_counts.keys()
            return "succeeded"

    class CountCategory(smach.State):
        def __init__(self, object_weight: Union[List[dict], None] = None):
            smach.State.__init__(
                self,
                outcomes=["succeeded", "failed"],
                input_keys=["object_dict"],
                output_keys=["category_dict", "detections_categories"],
            )
            self.object_weight = object_weight

        def count_category(self, object_dictionary, count_object):
            category_count = {category: 0 for category in object_dictionary.keys()}
            for category, items in object_dictionary.items():
                for obj in count_object.keys():
                    if obj in items:
                        category_count[category] += count_object[obj]
            return category_count

        def execute(self, userdata):
            detected_objects = userdata.object_dict
            counts = self.count_category(self.object_weight, detected_objects)
            category_counts = {
                key: value for key, value in counts.items() if value != 0
            }
            print("finished counting")
            print(category_counts)
            userdata.category_dict = category_counts
            userdata.detections_categories = category_counts.keys()
            return "succeeded"

    class ObjectWeight(smach.State):
        def __init__(self, object_weight: Union[List[dict], None] = None):
            smach.State.__init__(
                self,
                outcomes=["succeeded", "failed"],
                input_keys=["detections_types"],
                output_keys=["sorted_weights"],
            )
            self.object_weight = object_weight

        def get_weight(self, detections, average_weights):
            """
            rank the weight of a list of objects.

            :param detections: List of detection tuples (object_type, position)
            :return: a list of the weight rank of object in a category(max->min)
            """
            weight_dict = {}
            for category, items in average_weights.items():
                for i in detections:
                    if i in items:
                        weight = items[i]
                        weight_dict[i] = weight

            return weight_dict

        def execute(self, userdata):
            weights_dict = self.get_weight(
                userdata.detections_types, self.object_weight
            )
            sorted_weights = sorted(
                weights_dict.items(), key=lambda item: item[1], reverse=True
            )
            userdata.sorted_weights = sorted_weights
            print(sorted_weights)
            return "succeeded"

    class ObjectSize(smach.State):
        def __init__(self):
            smach.State.__init__(
                self,
                outcomes=["succeeded", "failed"],
                input_keys=["object_dict", "detections_3d"],
                output_keys=["sorted_size"],
            )

        def property_size_calculation(self, detections, result):
            """
            calculate the size of a list of objects using bounding box.
            :param detections: List of detection tuples (object_type, position)
            :return: a list of the size of each object in a category
            """

            area = dict()
            for i in detections:
                for object in result.detected_objects:
                    if i == object.name:
                        area[i] = object.xywh[2] * object.xywh[3]
            return area

        def execute(self, userdata):
            detections_types = userdata.object_dict.keys()
            area_dict = self.property_size_calculation(
                detections_types, userdata.detections_3d
            )
            sorted_size = sorted(
                area_dict.items(), key=lambda item: item[1], reverse=True
            )
            userdata.sorted_size = sorted_size
            print(sorted_size)
            return "succeeded"

    def __init__(
        self,
        area_polygon: Polygon,
        depth_topic: str = "/xtion/depth_registered/points",
        model: str = "yolov8n-seg.pt",
        filter: Union[List[str], None] = None,
        confidence: float = 0.5,
        nms: float = 0.3,
        object: Union[List[dict], None] = None,
    ):
        smach.StateMachine.__init__(
            self,
            outcomes=["succeeded", "failed"],
            output_keys=["detections_3d", "object_dict"],
        )

        with self:
            smach.StateMachine.add(
                "DETECT_OBJECTS_3D",
                Detect3DInArea(
                    depth_topic=depth_topic,
                    model=model,
                    filter=filter,
                    confidence=confidence,
                    nms=nms,
                ),
                transitions={"succeeded": "COUNTOBJECTS", "failed": "failed"},
            )

            smach.StateMachine.add(
                "COUNTOBJECTS",
                self.CountObjectTypes(area_polygon),
                transitions={"succeeded": "COUNTCATEGORY", "failed": "failed"},
            )
            smach.StateMachine.add(
                "COUNTCATEGORY",
                self.CountCategory(object_weight=object),
                transitions={"succeeded": "GETWEIGHT", "failed": "failed"},
            )
            smach.StateMachine.add(
                "GETWEIGHT",
                self.ObjectWeight(object_weight=object),
                transitions={"succeeded": "GETSIZE", "failed": "failed"},
            )
            smach.StateMachine.add(
                "GETSIZE",
                self.ObjectSize(),
                transitions={"succeeded": "succeeded", "failed": "failed"},
            )


if __name__ == "__main__":
    import rospy
    from sensor_msgs.msg import PointCloud2

    rospy.init_node("test_object_comparison")
    weight = rospy.get_param("/Object_list/Object")

    polygon = Polygon([[-1, 0], [1, 0], [0, 1], [1, 1]])
    sm = ObjectComparison(Polygon(), filter=["bottle", "cup", "cola"], object=weight)
    sm.userdata.pcl_msg = rospy.wait_for_message(
        "/xtion/depth_registered/points", PointCloud2
    )
    sm.execute()
