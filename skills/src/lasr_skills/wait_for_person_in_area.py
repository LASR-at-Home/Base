import smach

from lasr_skills import Detect3DInArea

from shapely.geometry.polygon import Polygon


class WaitForPersonInArea(smach.StateMachine):

    class CheckForPerson(smach.State):

        def __init__(self):
            smach.State.__init__(
                self, outcomes=["done", "not_done"], input_keys=["detections_3d"]
            )

        def execute(self, userdata):
            if len(userdata.detections_3d):
                return "done"
            else:
                return "not_done"

    def __init__(self, area_polygon: Polygon):
        smach.StateMachine.__init__(
            self,
            outcomes=["succeeded", "failed"],
            output_keys=["detections_3d"],
        )

        with self:
            smach.StateMachine.add(
                "DETECT_PEOPLE_3D",
                Detect3DInArea(area_polygon=area_polygon, filter=["person"]),
                transitions={"succeeded": "CHECK_FOR_PERSON", "failed": "failed"},
            )
            smach.StateMachine.add(
                "CHECK_FOR_PERSON",
                self.CheckForPerson(),
                transitions={"done": "succeeded", "not_done": "DETECT_PEOPLE_3D"},
            )
