#!/usr/bin/env python3
import smach
from lasr_skills import Detect3DInArea
from lasr_skills import Detect3D
from lasr_skills import Detect
from shapely.geometry.polygon import Polygon
from typing import List, Union

class ObjectComparison(smach.StateMachine):
    class CountObjectTypes(smach.State):
        def __init__(self, area_polygon: Polygon):
            smach.State.__init__(self, outcomes=['succeeded', 'failed'], input_keys=['detections_3d'], output_keys=['detections_types','object_dict'])

        def count_types(self, detections):
            """
            Count the number of different object types in detections.

            :param detections: List of detection tuples (object_type, position)
            :return: Dictionary with object types as keys and counts as values
            """
            # userdata.detections_3d = [(detection, point),(detection, point),(detection, point)]
            print("we're counting")
            object_counts = {}
            for detection in detections:
                object_type = detection.name
            
                if len(object_counts.keys()) == 0:
                    object_counts[object_type] = 1
                else:
                    if object_type in object_counts.keys():
                        object_counts[object_type] += 1
                    else:
                        print("yay")
                        object_counts[object_type] = 1
            return object_counts

        def execute(self, userdata):
            print("we're doing cout types")
            print(userdata.detections_3d)
            filtered_detections = userdata.detections_3d # the outcome of the 3d detections
            rospy.loginfo(filtered_detections)
            rospy.loginfo(type(filtered_detections.detected_objects[0]))
            object_counts = self.count_types(filtered_detections.detected_objects)
            print("finished counting")
            print(object_counts)
            userdata.object_dict = object_counts # 1.Tell me how many of an object (or object category) are on a placement location.
            userdata.detections_types = object_counts.keys() # 2.Find a (one or three) category of object in a room.
            return 'succeeded'
    
    class ObjectWeight(smach.State):
        def __init__(self):
            smach.State.__init__(self, outcomes=['succeeded', 'failed'], input_keys=['detections_types'], output_keys=['sorted_weights'])

        def get_weight(self, detections):
            """
            rank the weight of a list of objects.

            :param detections: List of detection tuples (object_type, position)
            :return: a list of the weight rank of object in a category(max->min)
            """
            average_weights = {
                    "sponge": 10,
                    "cleanser": 850,
                    "tennis ball": 58,
                    "soccer ball": 430,
                    "dice": 9,
                    "baseball":145,
                    "rubiks_cube": 100,
                    "apple": 182,
                    "orange": 131,
                    "pear":178,
                    "banana":118,
                    "strawberry":12,
                    "lemon":100,
                    "peach":150,
                    "plum":78,
                    "bottle":500,
                    "cup":300
                }
            weight_dict = {}#
            for i in detections:
                weight = average_weights[i]
                weight_dict[i] = weight
            
            return weight_dict
        
        def execute(self, userdata):
            print("we're doing weight")
            weights_dict = self.get_weight(userdata.detections_types)
            sorted_weights = sorted(weights_dict.items(), key=lambda item: item[1], reverse= True)
            userdata.sorted_weights = sorted_weights
            print(sorted_weights)
            return 'succeeded'

    class ObjectSize(smach.State):
        def __init__(self):
            smach.State.__init__(self, outcomes=['succeeded', 'failed'], input_keys=['object_dict', 'detections_3d'], output_keys=['sorted_size'])

        def property_size_calculation(self, detections, result):
            """
            calculate the size of a list of objects using bounding box.
            :param detections: List of detection tuples (object_type, position)
            :return: a list of the size of each object in a category 
            """
        # userdata.detections_3d = [(detection, point),(detection, point),(detection, point)]
        # the detection includes the size of the bounding box and the center point of the box
            area = dict()
            for i in detections:
                for object in result.detected_objects:
                    if i == object.name:
                        area[i] = object.xywh[2]*object.xywh[3]
            return area  
        
        def execute(self, userdata):
            # got the area dic of the detected objects
            # rank the size of each object
            print("we're doing size")
            detections_types = userdata.object_dict.keys()
            area_dict = self.property_size_calculation(detections_types, userdata.detections_3d)
            sorted_size = sorted(area_dict.items(), key=lambda item: item[1], reverse = True)
            userdata.sorted_size = sorted_size
            print(sorted_size)
            return 'succeeded'
        
    def __init__(
        self,
        area_polygon: Polygon,
        depth_topic: str = "/xtion/depth_registered/points",
        model: str = "yolov8n-seg.pt",
        filter: Union[List[str], None] = None,
        confidence: float = 0.5,
        nms: float = 0.3
    ):
        smach.StateMachine.__init__(
            self,
            outcomes=["succeeded", "failed"],
            output_keys=["detections_3d"],
        )

        with self:
            smach.StateMachine.add(
                "DETECT_OBJECTS_3D",
                Detect3D(
                    depth_topic=depth_topic,
                    model=model,
                    filter=filter,
                    confidence=confidence,
                    nms=nms,
                ),
                transitions={"succeeded": "DETECTIONS3D", "failed": "failed"},
            )
            # smach.StateMachine.add(
            #     "FILTER_DETECTIONS",
            #     Detect3DInArea(area_polygon),
            #     transitions={"succeeded": 'DETECTIONS3D', "failed": "failed"},
            # )
            smach.StateMachine.add('DETECTIONS3D', self.CountObjectTypes(area_polygon), transitions={'succeeded' : 'COUNTOBJECTS', 'failed' : 'failed'})
            smach.StateMachine.add('COUNTOBJECTS', self.ObjectWeight(), transitions={'succeeded' : 'GETWEIGHT', 'failed' : 'failed'})
            smach.StateMachine.add('GETWEIGHT', self.ObjectSize(), transitions={'succeeded' : 'succeeded', 'failed' : 'failed'})

if __name__ == "__main__":
    import rospy
    from sensor_msgs.msg import PointCloud2

    rospy.init_node("test_object_comparison")
    # polygon = Polygon(
    #     # [[3.45, -3.74],
    #     # [3.72, -4.78],
    #     # [2.4, -5.14],
    #     # [2.13, -3.92]]
    #     [[-1., 0,], [1,0], [0, 1], [1,1]]
    # )
    sm = ObjectComparison(Polygon(), filter=["bottle", "cup"])
    sm.userdata.pcl_msg = rospy.wait_for_message("/xtion/depth_registered/points", PointCloud2)
    sm.execute()
