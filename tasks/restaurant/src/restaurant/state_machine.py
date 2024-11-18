import navigation_helpers
import rospy
import smach
import smach_ros
from geometry_msgs.msg import Point, Pose, PoseStamped, PoseWithCovarianceStamped
from lasr_skills import (
    AskAndListen,
    GoToLocation,
    HandoverObject,
    PlayMotion,
    Rotate,
    Say,
    Wait,
)
from restaurant.speech.speech_handlers import handle_speech
from restaurant.states import Survey
from std_msgs.msg import Empty
from std_srvs.srv import Empty as EmptySrv
from std_srvs.srv import Trigger


class ApproachPerson(smach.StateMachine):

    class ComputeApproachPose(smach.State):

        def __init__(self):
            super().__init__(
                outcomes=["succeeded", "failed"],
                input_keys=["person_point"],
                output_keys=["customer_approach_pose"],
            )

        def execute(self, userdata):

            robot_pose_with_covariance = rospy.wait_for_message(
                "/robot_pose", PoseWithCovarianceStamped
            )
            robot_pose = PoseStamped(
                pose=robot_pose_with_covariance.pose.pose,
                header=robot_pose_with_covariance.header,
            )
            target_pose = PoseStamped(
                pose=Pose(
                    position=userdata.person_point,
                    orientation=robot_pose.pose.orientation,
                ),
                header=robot_pose.header,
            )

            approach_pose = navigation_helpers.get_approach_pose_on_radius(
                robot_pose, target_pose, 1.0
            )

            if approach_pose is None:
                approach_pose = navigation_helpers.get_pose_on_path(
                    robot_pose, target_pose, 1.5
                )

            if approach_pose is None:
                return "failed"

            approach_pose.pose.orientation = navigation_helpers.compute_face_quat(
                approach_pose.pose,
                target_pose.pose,
            )
            userdata.customer_approach_pose = approach_pose.pose
            return "succeeded"

    def __init__(self):
        super().__init__(
            outcomes=["succeeded", "failed"],
            input_keys=["person_point"],
            output_keys=["customer_approach_pose"],
        )

        with self:
            smach.StateMachine.add(
                "COMPUTE_APPROACH_POSE",
                self.ComputeApproachPose(),
                transitions={"succeeded": "GO_TO_PERSON", "failed": "failed"},
            )

            smach.StateMachine.add(
                "GO_TO_PERSON",
                GoToLocation(),
                transitions={"succeeded": "succeeded", "failed": "failed"},
                remapping={"location": "customer_approach_pose"},
            )


class Restaurant(smach.StateMachine):

    def __init__(
        self, bar_pose_map: Pose, unmapped: bool = False, dem_manipulation: bool = True
    ) -> None:
        super().__init__(outcomes=["succeeded", "failed"])

        with self:
            smach.StateMachine.add(
                "WAIT_START",
                smach_ros.MonitorState(
                    "/restaurant/start",
                    Empty,
                    lambda _ud, _msg: False,
                ),
                transitions={
                    "valid": "WAIT_START",
                    "preempted": "WAIT_START",
                    "invalid": "SAY_START",
                },
            )

            smach.StateMachine.add(
                "SAY_START",
                Say(text="Start of Restaurant task."),
                transitions={
                    "succeeded": "SURVEY" if not unmapped else "ROTATE_360",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )

            if unmapped:
                smach.StateMachine.add(
                    "ROTATE_360", Rotate(angle=360), transitions={"succeeded": "SURVEY"}
                )

            smach.StateMachine.add(
                "SURVEY",
                Survey(),
                transitions={
                    "customer_found": "GO_TO_CUSTOMER",
                    "customer_not_found": "SURVEY",
                },
            )

            smach.StateMachine.add(
                "GO_TO_CUSTOMER",
                ApproachPerson(),
                transitions={"succeeded": "TAKE_ORDER", "failed": "failed"},
            )

            smach.StateMachine.add(
                "TAKE_ORDER",
                AskAndListen(
                    tts_phrase="Hello, I'm TIAGo. What can I get for you today?"
                ),
                remapping={"transcribed_speech": "order_str"},
                transitions={
                    "succeeded": "PROCESS_ORDER",
                    "failed": "TAKE_ORDER_AGAIN",
                },
            )

            @smach.cb_interface(
                input_keys=["order_str"],
                output_keys=["order_str"],
                outcomes=["succeeded", "failed"],
            )
            def speech_postprocess_cb(ud):
                parsed_order, success = handle_speech(ud.order_str, False)
                if not success:
                    return "failed"
                order_string = ", ".join(
                    [
                        f"{count} {item if count == 1 or item.endswith('s') else item+'s'}"
                        for count, item in parsed_order
                    ]
                )
                ud.order_str = order_string
                return "succeeded"

            smach.StateMachine.add(
                "PROCESS_ORDER",
                smach.CBState(speech_postprocess_cb),
                transitions={"succeeded": "SAY_ORDER", "failed": "TAKE_ORDER_AGAIN"},
            )

            smach.StateMachine.add(
                "TAKE_ORDER_AGAIN",
                AskAndListen(
                    tts_phrase="Sorry, I didn't understand that. Can you state your order again?"
                ),
                remapping={"transcribed_speech": "order_str"},
                transitions={"succeeded": "PROCESS_ORDER", "failed": "failed"},
            )

            smach.StateMachine.add(
                "SAY_ORDER",
                Say(format_str="Your order is: {}. I will deliver it shortly."),
                remapping={"placeholders": "order_str"},
                transitions={
                    "succeeded": "GO_TO_BAR",
                    "preempted": "failed",
                    "aborted": "failed",
                },
            )

            smach.StateMachine.add(
                "GO_TO_BAR",
                GoToLocation(location=bar_pose_map),
                transitions={"succeeded": "REQUEST_ITEMS", "failed": "failed"},
            )

            smach.StateMachine.add(
                "REQUEST_ITEMS",
                Say(format_str="Please get me {}"),
                remapping={"placeholders": "order_str"},
                transitions={
                    "succeeded": "SAY_TURN" if dem_manipulation else "SAY_WAIT",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )

            if dem_manipulation:
                smach.StateMachine.add(
                    "SAY_TURN",
                    Say(text="I will turn around for you to load the order."),
                    transitions={
                        "succeeded": "ROTATE_LOAD",
                        "aborted": "failed",
                        "preempted": "failed",
                    },
                )
                smach.StateMachine.add(
                    "ROTATE_LOAD",
                    Rotate(angle=180.0),
                    transitions={"succeeded": "WAIT_LOAD"},
                )
                smach.StateMachine.add(
                    "WAIT_LOAD",
                    Wait(5),
                    transitions={"succeeded": "RETURN_TO_CUSTOMER", "failed": "failed"},
                )
            else:
                smach.StateMachine.add(
                    "SAY_WAIT",
                    Say(text="I will give you some time to prepare the order."),
                    transitions={
                        "succeeded": "WAIT_PREPARE",
                        "aborted": "failed",
                        "preempted": "failed",
                    },
                )
                smach.StateMachine.add(
                    "WAIT_PREPARE",
                    Wait(5),
                    transitions={"succeeded": "GRASP_OBJECT", "failed": "failed"},
                )
                smach.StateMachine.add(
                    "GRASP_OBJECT",
                    smach_ros.ServiceState(
                        "tiago_kcl_moveit_grasp/grasp_object", Trigger
                    ),
                    transitions={
                        "succeeded": "RETURN_TO_CUSTOMER",
                        "aborted": "failed",
                        "preempted": "failed",
                    },
                )

            smach.StateMachine.add(
                "RETURN_TO_CUSTOMER",
                GoToLocation(),
                remapping={"location": "customer_approach_pose"},
                transitions={
                    "succeeded": (
                        "SAY_TAKE_ORDER" if dem_manipulation else "HANDOVER_ORDER"
                    ),
                    "failed": "failed",
                },
            )

            if dem_manipulation:

                smach.StateMachine.add(
                    "SAY_TAKE_ORDER",
                    Say(
                        text="Here is your order, I will turn around for you to unload it."
                    ),
                    transitions={
                        "succeeded": "ROTATE_UNLOAD",
                        "aborted": "failed",
                        "preempted": "failed",
                    },
                )

                smach.StateMachine.add(
                    "ROTATE_UNLOAD",
                    Rotate(angle=180.0),
                    transitions={"succeeded": "WAIT_UNLOAD"},
                )

                smach.StateMachine.add(
                    "WAIT_UNLOAD",
                    Wait(5),
                    transitions={"succeeded": "GO_TO_SURVEY", "failed": "failed"},
                )

            else:
                smach.StateMachine.add(
                    "HANDOVER_ORDER",
                    HandoverObject("order"),
                    transitions={"succeeded": "GO_TO_SURVEY", "failed": "failed"},
                )

            smach.StateMachine.add(
                "GO_TO_SURVEY",
                GoToLocation(location=bar_pose_map),
                transitions={"succeeded": "SURVEY", "failed": "failed"},
            )
