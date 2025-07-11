import smach
import rospy
import smach_ros

from control_msgs.msg import (
    FollowJointTrajectoryActionGoal,
    FollowJointTrajectoryAction,
    JointTrajectoryPoint,
)
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Header

from lasr_skills import DetectAllInPolygonSensorData, LookToPoint, Say
from shapely import Polygon as ShapelyPolygon
from collections import defaultdict


class ScanShelves(smach.StateMachine):

    def __init__(self):
        """
        Scan each shelf to determine the category of the objects from:
        - cleaning_supplies
        - drinks
        - food
        - decorations
        - fruits
        - snacks
        - dishes
        """
        super().__init__(
            outcomes=["succeeded", "failed"],
            output_keys=["shelf_data"],
            input_keys=["selected_object"],
        )

        with self:

            # smach.StateMachine.add(
            #     "LOOK_AT_SHELF",
            #     LookToPoint(
            #         pointstamped=PointStamped(
            #             point=Point(
            #                 **rospy.get_param(
            #                     "/storing_groceries/cabinet/shelves/bottom/look_point"
            #                 )
            #             ),
            #             header=Header(frame_id="map"),
            #         )
            #     ),
            #     transitions={
            #         "succeeded": "DETECT_OBJECTS",
            #         "aborted": "failed",
            #         "timed_out": "failed",
            #     },
            # )

            # smach.StateMachine.add(
            #     "DETECT_OBJECTS",
            #     DetectAllInPolygonSensorData(
            #         ShapelyPolygon(rospy.get_param("/storing_groceries/table/polygon")),
            #         object_filter=[
            #             k for k in rospy.get_param("/storing_groceries/objects")
            #         ],
            #         min_coverage=0.9,
            #         min_confidence=0.1,
            #         z_axis=0.8,
            #         model="lasr.pt",
            #     ),
            #     transitions={"succeeded": "CLASSIFY_SHELF", "failed": "failed"},
            # )

            # smach.StateMachine.add(
            #     "CLASSIFY_SHELF",
            #     smach.CBState(
            #         self._classify_shelf,
            #         output_keys=["shelf_category"],
            #         input_keys=["detected_objects"],
            #         outcomes=["succeeded", "failed"],
            #     ),
            #     transitions={"succeeded": "SAY_CLASSIFICATION", "failed": "failed"},
            # )

            # smach.StateMachine.add(
            #     "SAY_CLASSIFICATION",
            #     Say(format_str="This shelf contains {}"),
            #     remapping={"placeholders": "shelf_category"},
            #     transitions={
            #         "succeeded": "succeeded",
            #         "preempted": "failed",
            #         "aborted": "failed",
            #     },
            # )

            # smach.StateMachine.add(
            #     "MATCH_CLASSIFICATION",
            #     smach.CBState(
            #         self._match_classification,
            #         output_keys=["say_str"],
            #         input_keys=["shelf_category", "selected_object"],
            #         outcomes=["succeeded"],
            #     ),
            #     transitions={"succeeded": "SAY_CLASSIFICATION_STR"},
            # )

            # smach.StateMachine.add(
            #     "SAY_CLASSIFICATION_STR",
            #     Say(),
            #     remapping={"text": "say_str"},
            #     transitions={
            #         "succeeded": "succeeded",
            #         "preempted": "failed",
            #         "aborted": "failed",
            #     },
            # )

            self.userdata.shelf_data = {}

            shelf_iterator = smach.Iterator(
                outcomes=["succeeded", "failed"],
                it=rospy.get_param("/storing_groceries/cabinet/shelves"),
                it_label="shelf_id",
                exhausted_outcome="succeeded",
                input_keys=[],
                output_keys=[],
            )

            with shelf_iterator:
                container_sm = smach.StateMachine(
                    outcomes=["continue", "succeeded", "failed"],
                    input_keys=["shelf_id"],
                )

                with container_sm:

                    smach.StateMachine.add(
                        "GET_SHELF_DATA",
                        smach.CBState(
                            self._get_shelf_data,
                            output_keys=[
                                "torso_height",
                                "look_point",
                                "polygon",
                                "z_sweep_min",
                                "z_sweep_max",
                            ],
                            input_keys=["shelf_id"],
                        ),
                        transitions={"succeeded": "succeeded"},
                    )

                    smach.StateMachine.add(
                        "ADJUST_TORSO",
                        smach_ros.SimpleActionState(
                            "/torso_controller/follow_joint_trajectory",
                            FollowJointTrajectoryAction,
                            goal_cb=self._adjust_torso_goal_cb,
                            output_keys=[],
                            input_keys=["shelf_id"],
                        ),
                        transitions={
                            "succeeded": "DETECT_OBJECTS",
                            "preempted": "failed",
                            "aborted": "failed",
                        },
                    )

                    smach.StateMachine.add(
                        "DETECT_OBJECTS",
                        DetectAllInPolygonSensorData(
                            object_filter=[
                                k for k in rospy.get_param("/storing_groceries/objects")
                            ],
                            min_coverage=0.9,
                            min_confidence=0.1,
                            model="lasr.pt",
                        ),
                        transitions={"succeeded": "SELECT_OBJECT", "failed": "failed"},
                    )

    def _adjust_torso_goal_cb(self, userdata, goal):
        goal = FollowJointTrajectoryActionGoal()
        goal.trajectory.joint_names = ["torso_lift_joint"]
        point = JointTrajectoryPoint()
        point.positions = [userdata.torso_height]
        point.time_from_start = rospy.Duration(1)
        goal.trajectory.points.append(point)
        return goal

    def _get_shelf_data(self, userdata):
        userdata.torso_height = rospy.get_param(
            f"/storing_groceries/cabinet/shelves/{userdata.shelf_id}/torso_lift_joint"
        )
        userdata.look_point = rospy.get_param(
            f"/storing_groceries/cabinet/shelves/{userdata.shelf_id}/look_point"
        )
        userdata.polygon = ShapelyPolygon(
            rospy.get_param(
                f"/storing_groceries/cabinet/shelves/{userdata.shelf_id}/polygon"
            )
        )
        userdata.z_sweep_min = rospy.get_param(
            f"/storing_groceroes/cabinet/shelves/{userdata.shelf_id}/z_min"
        )
        userdata.z_sweep_max = rospy.get_param(
            f"/storing_groceries/cabinet/shelves/{userdata.shelf_idz}z_max"
        )

        return "succeeded"

    def _classify_shelf(self, userdata):

        category_counts = defaultdict(int)

        if not userdata.detected_objects:
            return "failed"

        for obj, _ in userdata.detected_objects:
            category = rospy.get_param(
                f"/storing_groceries/objects/{obj.name}/category"
            )
            category_counts[category] += 1

        shelf_category = max(category_counts, lambda key: category_counts[key])
        userdata.shelf_category = shelf_category

        return "succeeded"

    def _match_classification(self, userdata):
        object_category = rospy.get_param(
            f"/storing_groceries/objects/{userdata.selected_object[0].name}/category"
        )
        if object_category != userdata.shelf_category:
            userdata.say_str = f"{userdata.selected_object[0].name} is a {object_category}. This shelf contains {userdata.shelf_category}. Since it's the only shelf, I will place it here anwyay."
        else:
            userdata.say_str = f"{userdata.selected_object[0].name} is a {object_category}. This shelf contains {userdata.shelf_category}, so I will place the {userdata.selected_object[0].name} here."
        return "succeeded"
