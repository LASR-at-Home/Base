import rospy
import smach
import smach_ros
from lasr_skills import (
    WaitForPerson,
    Say,
    DetectGesture,
    ReceiveObject,
    HandoverObject,
    GoToLocation,
)
from lasr_skills.vision import GetCroppedImage
from lasr_skills import PlayMotion
from lasr_person_following.msg import FollowAction
from leg_tracker.srv import (
    InitialisePersonWithVision,
    InitialisePersonWithVisionRequest,
)

from std_msgs.msg import Empty
from pal_navigation_msgs.srv import Acknowledgment, AcknowledgmentRequest
from std_srvs.srv import Empty as EmptySrv

from geometry_msgs.msg import Pose, Point, Quaternion

from pal_startup_msgs.srv import (
    StartupStart,
    StartupStop,
    StartupStartRequest,
    StartupStopRequest,
)


class CarryMyLuggage(smach.StateMachine):
    class ProcessPointingDirection(smach.State):
        def __init__(self):
            smach.State.__init__(
                self,
                outcomes=["succeeded", "failed"],
                input_keys=["gesture_detected"],
                output_keys=["pointing_direction"],
            )

        def execute(self, userdata):
            if userdata.gesture_detected == "pointing_to_the_left":
                userdata.pointing_direction = "left"
            elif userdata.gesture_detected == "pointing_to_the_right":
                userdata.pointing_direction = "right"
            else:
                return "failed"
            return "succeeded"

    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed"])
        with self:

            def wait_cb(ud, msg):
                rospy.loginfo("Received start signal")
                return False

            smach.StateMachine.add(
                "WAIT_START",
                smach_ros.MonitorState(
                    "/carry_my_luggage/start",
                    Empty,
                    wait_cb,
                ),
                transitions={
                    "valid": "WAIT_START",
                    "invalid": "STOP_HEAD_MANAGER",
                    "preempted": "WAIT_START",
                },
            )

            smach.StateMachine.add(
                "STOP_HEAD_MANAGER",
                smach_ros.ServiceState(
                    "/pal_startup_control/stop",
                    StartupStop,
                    request=StartupStopRequest("head_manager"),
                ),
                transitions={
                    "succeeded": "LOOK_CENTRE",
                    "aborted": "LOOK_CENTRE",
                    "preempted": "LOOK_CENTRE",
                },
            )

            smach.StateMachine.add(
                "LOOK_CENTRE",
                PlayMotion(motion_name="look_centre"),
                transitions={
                    "succeeded": "WAIT_FOR_PERSON",
                    "aborted": "WAIT_FOR_PERSON",
                    "preempted": "WAIT_FOR_PERSON",
                },
            )

            smach.StateMachine.add(
                "WAIT_FOR_PERSON",
                WaitForPerson(),
                transitions={
                    "succeeded": "SAY_POINT_TO_BAG",
                    "failed": "failed",
                },
            )

            smach.StateMachine.add(
                "SAY_POINT_TO_BAG",
                Say(text="Please point to the bag that you wish me to carry."),
                transitions={
                    "succeeded": "GET_IMAGE",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )

            smach.StateMachine.add(
                "GET_IMAGE",
                GetCroppedImage(object_name="person", method="closest", use_mask=True),
                transitions={
                    "succeeded": "DETECT_POINTING_DIRECTION",
                    "failed": "failed",
                },
            )

            smach.StateMachine.add(
                "DETECT_POINTING_DIRECTION",
                DetectGesture(),
                transitions={
                    "succeeded": "PROCESS_POINTING_DIRECTION",
                    "failed": "GET_IMAGE",
                    "missing_keypoints": "GET_IMAGE",
                },
            )

            smach.StateMachine.add(
                "PROCESS_POINTING_DIRECTION",
                CarryMyLuggage.ProcessPointingDirection(),
                transitions={
                    "succeeded": "SAY_BAG",
                    "failed": "GET_IMAGE",
                },
            )

            smach.StateMachine.add(
                "SAY_BAG",
                Say(format_str="I need you to give me the bag on your {}."),
                transitions={
                    "succeeded": "START_HEAD_MANAGER",
                    "aborted": "failed",
                    "preempted": "failed",
                },
                remapping={"placeholders": "pointing_direction"},
            )

            smach.StateMachine.add(
                "START_HEAD_MANAGER",
                smach_ros.ServiceState(
                    "/pal_startup_control/start",
                    StartupStart,
                    request=StartupStartRequest("head_manager", ""),
                ),
                transitions={
                    "succeeded": "RECEIVE_BAG",
                    "aborted": "RECEIVE_BAG",
                    "preempted": "RECEIVE_BAG",
                },
            )

            smach.StateMachine.add(
                "RECEIVE_BAG",
                ReceiveObject(object_name="bag", vertical=True),
                transitions={
                    "succeeded": "CLEAR_COSTMAPS",
                    "failed": "failed",
                },
            )

            # TODO: ensure it doesnt get stuck here, maybe concurrenr
            smach.StateMachine.add(
                "CLEAR_COSTMAPS",
                smach_ros.ServiceState(
                    "/move_base/clear_costmaps",
                    EmptySrv,
                ),
                transitions={
                    "succeeded": "GET_START_POSE",
                    "aborted": "GET_START_POSE",
                    "preempted": "GET_START_POSE",
                },
            )

            def start_pose_cb(ud):
                try:
                    ud.start_pose = rospy.wait_for_message("/robot_pose", PoseWithCovarianceStamped, timeout=rospy.Duration(5.0)).pose.pose
                except rospy.ROSException:
                    rospy.logerr("Failed to get robot pose")
                    ud.start_pose = Pose(position=Point(0., 0., 0.0), orientation=Quaternion(0., 0., 0., 1.))
                return "succeeded"
            smach.StateMachine.add(
                "GET_START_LOCATION",
                smach.CBState(
                    start_pose_cb,
                    outcomes=["succeeded"],
                    output_keys=["start_pose"],
                ),
                transitions={"succeeded": "SAY_STEP"}
            )

            smach.StateMachine.add(
                "SAY_STEP",
                Say(text="First walk slowly towards me and then I will follow you."),
                transitions={
                    "succeeded": "SAY_FOLLOW",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )
            smach.StateMachine.add(
                "SAY_FOLLOW",
                Say(text="I will follow you now."),
                transitions={
                    "succeeded": "FOLLOW",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )

            smach.StateMachine.add(
                "FOLLOW",
                smach_ros.SimpleActionState(
                    "follow_person",
                    FollowAction,
                ),
                transitions={
                    "succeeded": "SAY_HANDOVER",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )

            smach.StateMachine.add(
                "SAY_HANDOVER",
                Say(text="I will hand over the bag to you now."),
                transitions={
                    "succeeded": "HANDOVER",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )

            smach.StateMachine.add(
                "HANDOVER",
                HandoverObject(object_name="bag", vertical=True),
                transitions={
                    "succeeded": "succeeded",
                    "failed": "failed",
                },
            )

            smach.StateMachine.add(
                "SAY_GOING_BACK",
                Say(text="I am going back to the start position."),
                transitions={
                    "succeeded": "GO_TO_START",
                    "preempted": "GO_TO_START",
                    "aborted": "GO_TO_START",
                },
            )

            smach.StateMachine.add(
                "GO_TO_START", # todo: instead, get the start position within the state machine, and return to it at the end
                GoToLocation(),
                remapping={"location" : "start_pose"},
                transitions={"succeeded": "succeeded", "failed": "succeeded"},
            )
