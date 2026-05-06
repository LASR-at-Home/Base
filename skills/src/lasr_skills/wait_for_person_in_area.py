import smach_ros
import smach

from lasr_skills import Detect3DInArea

from shapely.geometry.polygon import Polygon

class CheckForPerson(smach_ros.RosState):
    def __init__(self, node):
        super().__init__(
            self, node=node, outcomes=["done", "not_done"], input_keys=["detections_3d"]
        )

    def execute(self, userdata):
        if len(userdata.detections_3d):
            return "done"
        else:
            return "not_done"

class WaitForPersonInArea(smach.StateMachine):
    def __init__(self, node, area_polygon_param: Polygon):
        smach.StateMachine.__init__(
            self,
            outcomes=["succeeded", "failed"],
            output_keys=["detections_3d"],
        )

        top_left = node.get_parameter('door_polygon').get_parameter_value('top_left')
        top_right = node.get_parameter('door_polygon').get_parameter_value('top_right')
        bottom_left = node.get_parameter('door_polygon').get_parameter_value('bottom_left')
        bottom_right = node.get_parameter('door_polygon').get_parameter_value('bottom_right')
        
        door_polygon = Polygon([top_left, top_right, bottom_left, bottom_right])

        with self:
            self.add(
                "DETECT_PEOPLE_3D",
                Detect3DInArea(node=node, area_polygon=door_polygon, filter=["person"], z_min=0.0, z_max=1.0),
                transitions={"succeeded": "CHECK_FOR_PERSON", "failed": "failed"},
            )
            self.add(
                "CHECK_FOR_PERSON",
                CheckForPerson(),
                transitions={"done": "succeeded", "not_done": "DETECT_PEOPLE_3D"},
            )