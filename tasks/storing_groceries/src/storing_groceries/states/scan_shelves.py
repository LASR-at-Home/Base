import smach
import rospy
import smach_ros

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Header

from lasr_skills import DetectAllInPolygonSensorData, LookToPoint, Say, Wait
from shapely import Polygon as ShapelyPolygon
from collections import defaultdict


class ScanShelves(smach.StateMachine):

    def __init__(self):
        super().__init__(
            outcomes=["succeeded", "failed"],
            output_keys=["shelf_data"],
            input_keys=["selected_object"],
        )

        with self:

            self.userdata.shelf_data = {}

            shelf_iterator = smach.Iterator(
                outcomes=["succeeded", "failed"],
                it=rospy.get_param("/storing_groceries/cabinet/shelves"),
                it_label="shelf_id",
                exhausted_outcome="succeeded",
                input_keys=["shelf_data"],
                output_keys=["shelf_data"],
            )

            with shelf_iterator:
                container_sm = smach.StateMachine(
                    outcomes=["continue", "succeeded", "failed"],
                    input_keys=["shelf_id", "shelf_data"],
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
                            outcomes=["succeeded"],
                        ),
                        transitions={"succeeded": "ADJUST_TORSO"},
                    )

                    smach.StateMachine.add(
                        "ADJUST_TORSO",
                        smach_ros.SimpleActionState(
                            "/torso_controller/follow_joint_trajectory",
                            FollowJointTrajectoryAction,
                            goal_cb=self._adjust_torso_goal_cb,
                            output_keys=[],
                            input_keys=["torso_height"],
                        ),
                        transitions={
                            "succeeded": "LOOK_AT_SHELF",
                            "preempted": "LOOK_AT_SHELF",
                            "aborted": "LOOK_AT_SHELF",  # always aborts?
                        },
                    )

                    smach.StateMachine.add(
                        "LOOK_AT_SHELF",
                        LookToPoint(),
                        transitions={
                            "succeeded": "DETECT_OBJECTS",
                            "aborted": "failed",
                            "timed_out": "failed",
                        },
                        remapping={"pointstamped": "look_point"},
                    )

                    smach.StateMachine.add(
                        "DETECT_OBJECTS",
                        DetectAllInPolygonSensorData(
                            object_filter=[
                                k for k in rospy.get_param("/storing_groceries/objects")
                            ],
                            min_coverage=1.0,
                            min_confidence=0.1,
                            model="lasr.pt",
                        ),
                        transitions={
                            "succeeded": "CLASSIFY_SHELF",
                            "failed": "CLASSIFY_SHELF",
                        },
                    )

                    smach.StateMachine.add(
                        "CLASSIFY_SHELF",
                        smach.CBState(
                            self._classify_shelf,
                            output_keys=["shelf_data"],
                            input_keys=["shelf_data", "detected_objects"],
                            outcomes=["succeeded", "failed"],
                        ),
                        transitions={
                            "succeeded": "continue",
                            "failed": "failed",
                        },
                    )

                shelf_iterator.set_contained_state(
                    "CONTAINER_STATE", container_sm, loop_outcomes=["continue"]
                )

            smach.StateMachine.add(
                "ITERATE_SHELVES",
                shelf_iterator,
                transitions={"succeeded": "succeeded"},
            )

    def _adjust_torso_goal_cb(self, userdata, goal):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ["torso_lift_joint"]
        point = JointTrajectoryPoint()
        point.positions = [userdata.torso_height]
        point.time_from_start = rospy.Duration(2.0)
        goal.trajectory.points.append(point)
        return goal

    def _get_shelf_data(self, userdata):
        userdata.torso_height = rospy.get_param(
            f"/storing_groceries/cabinet/shelves/{userdata.shelf_id}/torso_lift_joint"
        )
        userdata.look_point = PointStamped(
            point=Point(
                **rospy.get_param(
                    f"/storing_groceries/cabinet/shelves/{userdata.shelf_id}/look_point"
                )
            ),
            header=Header(frame_id="map"),
        )
        userdata.polygon = ShapelyPolygon(
            rospy.get_param(
                f"/storing_groceries/cabinet/shelves/{userdata.shelf_id}/polygon"
            )
        )
        userdata.z_sweep_min = rospy.get_param(
            f"/storing_groceries/cabinet/shelves/{userdata.shelf_id}/z_min"
        )
        userdata.z_sweep_max = rospy.get_param(
            f"/storing_groceries/cabinet/shelves/{userdata.shelf_id}/z_max"
        )

        return "succeeded"

    def _classify_shelf(self, userdata):

        category_counts = defaultdict(int)
        userdata.shelf_data["objects"] = [
            obj.name for obj, _ in userdata.detected_objects
        ]

        if not userdata.detected_objects:
            userdata.shelf_data["category"] = "empty"
            return "succeeded"

        for obj, _ in userdata.detected_objects:
            category = rospy.get_param(
                f"/storing_groceries/objects/{obj.name}/category"
            )
            category_counts[category] += 1

        shelf_category = max(category_counts, key=lambda key: category_counts[key])
        userdata.shelf_data["category"] = shelf_category

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
