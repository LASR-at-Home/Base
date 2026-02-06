import rospy
import smach
from shapely import Polygon as ShapelyPolygon

from lasr_skills import Rotate, DetectAllInPolygon, GoToLocation, Say

from storing_groceries.states import ComputeApproach


class GoToTable(smach.StateMachine):

    def __init__(self):
        super().__init__(
            outcomes=["succeeded", "failed"],
            input_keys=["table_approach_poses"],
            output_keys=["table_approach_poses", "table_pose"],
        )

        with self:

            smach.StateMachine.add(
                "GET_POSE",
                smach.CBState(
                    self._get_candidate_pose,
                    output_keys=["table_approach_pose", "table_approach_poses"],
                    outcomes=["succeeded", "failed"],
                    input_keys=["table_approach_poses"],
                ),
                transitions={"succeeded": "GO_TO_TABLE", "failed": "failed"},
            )

            smach.StateMachine.add(
                "GO_TO_TABLE",
                GoToLocation(),
                remapping={"location": "table_approach_pose"},
                transitions={"succeeded": "SET_TABLE_POSE", "failed": "GET_POSE"},
            )

            smach.StateMachine.add(
                "SET_TABLE_POSE",
                smach.CBState(
                    self._set_table_pose,
                    output_keys=["table_pose"],
                    outcomes=["succeeded"],
                    input_keys=["table_approach_pose"],
                ),
            )

    def _get_candidate_pose(self, userdata):
        if not userdata.table_approach_poses:
            return "failed"
        userdata.table_approach_pose = userdata.table_approach_poses.pop(0)
        return "succeeded"

    def _set_table_pose(self, userdata):
        userdata.table_pose = userdata.table_approach_pose
        return "succeeded"


class FindAndGoToTable(smach.StateMachine):

    def __init__(self):
        super().__init__(outcomes=["succeeded", "failed"], output_keys=["table_pose"])

        with self:

            smach.StateMachine.add(
                "ROTATE_90",
                Rotate(angle=-90.0),
                transitions={"succeeded": "SAY_LOOKING", "failed": "ROTATE_90"},
            )
            smach.StateMachine.add(
                "SAY_LOOKING",
                Say(text="I am looking for the table."),
                transitions={
                    "succeeded": "DETECT_TABLE",
                    "preempted": "DETECT_TABLE",
                    "aborted": "DETECT_TABLE",
                },
            )
            smach.StateMachine.add(
                "DETECT_TABLE",
                DetectAllInPolygon(
                    polygon=ShapelyPolygon(
                        rospy.get_param("/storing_groceries/table/search_polygon")
                    ),
                    object_filter=["dining table", "tv"],
                    min_confidence=0.05,
                ),
                transitions={"succeeded": "GET_TABLE_POSE", "failed": "DETECT_TABLE"},
            )
            smach.StateMachine.add(
                "GET_TABLE_POSE",
                smach.CBState(
                    self._get_table_pose,
                    output_keys=["table_poses"],
                    outcomes=["succeeded", "failed"],
                    input_keys=["detected_objects"],
                ),
                transitions={
                    "succeeded": "GET_CANDIDATE_POSES",
                    "failed": "DETECT_TABLE",
                },
            )
            smach.StateMachine.add(
                "GET_CANDIDATE_POSES",
                ComputeApproach(),
                transitions={
                    "succeeded": "GO_TO_TABLE",
                    "failed": "DETECT_TABLE",
                },
                remapping={"table_candidate_poses": "table_poses"},
            )

            smach.StateMachine.add(
                "GO_TO_TABLE",
                GoToTable(),
                transitions={"succeeded": "succeeded", "failed": "DETECT_TABLE"},
            )

    def _get_table_pose(self, userdata):
        table_poses = [
            obj.point
            for obj in userdata.detected_objects
            if obj.name in ["dining table", "tv"]
        ]
        if table_poses:
            userdata.table_poses = table_poses
            return "succeeded"
        return "failed"
