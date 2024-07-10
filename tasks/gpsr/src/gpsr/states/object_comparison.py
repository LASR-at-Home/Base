#!/usr/bin/env python3
import smach
from lasr_skills import Detect3DInArea, Say
from lasr_skills import Detect3D
from lasr_skills import Detect
from shapely.geometry.polygon import Polygon
from typing import List, Union, Dict
import rospy


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
            object_counts = {}
            for detection in detections:
                object_type = detection.name
                if object_type in object_counts:
                    object_counts[object_type] += 1
                else:
                    object_counts[object_type] = 1
            return object_counts

        def execute(self, userdata):
            filtered_detections = userdata.detections_3d
            rospy.loginfo(filtered_detections)
            object_counts = self.count_types(filtered_detections.detected_objects)
            userdata.object_dict = object_counts
            userdata.detections_types = list(object_counts.keys())
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
            userdata.category_dict = category_counts
            userdata.detections_categories = list(category_counts.keys())
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
            area = dict()
            for i in detections:
                for object in result.detected_objects:
                    if i == object.name:
                        area[i] = object.xywh[2] * object.xywh[3]
            return area

        def execute(self, userdata):
            detections_types = list(userdata.object_dict.keys())
            area_dict = self.property_size_calculation(
                detections_types, userdata.detections_3d
            )
            sorted_size = sorted(
                area_dict.items(), key=lambda item: item[1], reverse=True
            )
            userdata.sorted_size = sorted_size
            return "succeeded"

    class DecideOperation(smach.State):
        def __init__(self):
            smach.State.__init__(
                self,
                outcomes=["do_count", "do_weight", "do_size", "failed"],
                input_keys=["operation_label"],
            )

        def execute(self, userdata):
            if userdata.operation_label == "count":
                return "do_count"
            elif userdata.operation_label == "weight":
                return "do_weight"
            elif userdata.operation_label == "size":
                return "do_size"
            else:
                return "failed"

    class SayResult(smach.State):
        def __init__(self):
            smach.State.__init__(
                self,
                outcomes=["succeeded", "failed"],
                input_keys=[
                    "operation_label",
                    "detections_types",
                    "detections_categories",
                    "sorted_size",
                    "sorted_weights",
                ],
                output_keys=["say_text"],
            )

        def execute(self, userdata):
            try:
                if userdata.operation_label == "count":
                    object_count = len(userdata.detections_categories)
                    userdata.say_text = f"There are {object_count} objects"
                elif userdata.operation_label == "weight":
                    heaviest_object = userdata.sorted_weights[0][0]
                    userdata.say_text = f"The heaviest object is {heaviest_object}"
                elif userdata.operation_label == "size":
                    biggest_object = userdata.sorted_size[0][0]
                    userdata.say_text = f"The biggest object is {biggest_object}"
                else:
                    return "failed"
                return "succeeded"
            except Exception as e:
                rospy.logerr(str(e))
                return "failed"

    def __init__(
        self,
        area_polygon: Polygon,
        operation_label: str,
        depth_topic: str = "/xtion/depth_registered/points",
        model: str = "yolov8n-seg.pt",
        filter: Union[List[str], None] = None,
        confidence: float = 0.5,
        nms: float = 0.3,
        object_weight: Union[List[dict], None] = None,
    ):
        smach.StateMachine.__init__(
            self,
            outcomes=["succeeded", "failed"],
            output_keys=["detections_3d", "object_dict", "say_text"],
        )

        # Set the operation label in the userdata
        self.userdata.operation_label = operation_label

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
                transitions={"succeeded": "DECIDE_OPERATION", "failed": "failed"},
            )

            smach.StateMachine.add(
                "DECIDE_OPERATION",
                self.DecideOperation(),
                transitions={
                    "do_count": "COUNTOBJECTS",
                    "do_weight": "GETWEIGHT",
                    "do_size": "GETSIZE",
                    "failed": "failed",
                },
            )

            smach.StateMachine.add(
                "COUNTOBJECTS",
                self.CountObjectTypes(area_polygon),
                transitions={"succeeded": "COUNTCATEGORY", "failed": "failed"},
            )

            smach.StateMachine.add(
                "COUNTCATEGORY",
                self.CountCategory(object_weight=object_weight),
                transitions={"succeeded": "SAY_RESULT", "failed": "failed"},
            )

            smach.StateMachine.add(
                "GETWEIGHT",
                self.ObjectWeight(object_weight=object_weight),
                transitions={"succeeded": "SAY_RESULT", "failed": "failed"},
            )

            smach.StateMachine.add(
                "GETSIZE",
                self.ObjectSize(),
                transitions={"succeeded": "SAY_RESULT", "failed": "failed"},
            )

            smach.StateMachine.add(
                "SAY_RESULT",
                self.SayResult(),
                transitions={"succeeded": "SAY", "failed": "failed"},
            )

            smach.StateMachine.add(
                "SAY",
                Say(),
                transitions={"succeeded": "succeeded", "failed": "failed"},
                remapping={"text": "say_text"},
            )


# if __name__ == "__main__":
#     import rospy
#     from sensor_msgs.msg import PointCloud2

#     rospy.init_node("test_object_comparison")
#     weight = rospy.get_param("/Object_list/Object")

#     polygon = Polygon([[-1, 0], [1, 0], [0, 1], [1, 1]])
#     sm = ObjectComparison(Polygon(), filter=["bottle", "cup", "cola"], object=weight)
#     sm.userdata.pcl_msg = rospy.wait_for_message(
#         "/xtion/depth_registered/points", PointCloud2
#     )
#     sm.execute()
