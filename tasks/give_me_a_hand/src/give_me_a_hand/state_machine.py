from typing import List, Tuple

import rospy
import smach
import smach_ros
from geometry_msgs.msg import Point, PointStamped, Pose
from lasr_skills import Say, GoToLocation
from lasr_vision_msgs.srv import Recognise
from give_me_a_hand.states import WaitDoorOpen, FindOperators
from shapely.geometry import Polygon
from std_msgs.msg import Empty, Header



class GiveMeAHand(smach.StateMachine):
    def __init__(
        self,
    ):
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed"])

        # Strategy1. Give every possible pose and pick close, wider to narrow (exclude impossible)
        # Strategy2. If location changes. Yolo detection?
        with self:
            # def wait_cb(ud, msg):
            #     rospy.loginfo("Received start signal")
            #     return False

            # smach.StateMachine.add(
            #     "WAIT_START",
            #     smach_ros.MonitorState(
            #         "/give_me_a_hand/start",
            #         Empty,
            #         wait_cb,
            #     ),
            #     transitions={
            #         "valid": "WAIT_START",
            #         "invalid": "SAY_START",
            #         "preempted": "WAIT_START",
            #     },
            # )

            self.pose = rospy.get_param(f"/arena/start_pose/{waiting_}", "unknown")
            self.pose = self.convert_yaml_to_pose(pose)
        
            smach.StateMachine.add(
                "SAY_START",
                Say(text="Ready to start"),
                transitions={
                    "succeeded": "GO_TO_WAITING_AREA",
                    "aborted": "GO_TO_WAITING_AREA",
                    "preempted": "GO_TO_WAITING_AREA",
                },
            )

            # self.go_to_waiting_area() #may be we don't need this

            """
            WaitDoorOpen
            """

            smach.StateMachine.add(
                "SAY_WAIT",
                Say(text="Waiting for door to open"),
                transitions={
                    "succeeded": "WAIT_DOOR_OPEN",
                    "aborted": "WAIT_DOOR_OPEN",
                    "preempted": "WAIT_DOOR_OPEN",
                },
            )

            smach.StateMachine.add(
                "WAIT_DOOR_OPEN",
                WaitDoorOpen(),
                transitions={
                    "succeeded": "FIND_OPERATORS",
                    "aborted": "FIND_OPERATORS",
                    "preempted": "FIND_OPERATORS",
                },
            )

            """
            FIND_OPERATORS_LOOP
            """

            smach.StateMachine.add(
                "FIND_OPERATORS",
                FindOperators(),
                transitions={
                    "succeeded": "GO_TO_OPERATORS",
                    "failed": "GO_TO_OPERATORS",
                    "escape": "SAY_FINISHED",
                },
            )

            smach.StateMachine.add(
                f"GO_TO_OPERATORS",
                GoToLocation(),
                remapping={"location": "customer_approach_pose"},
                transitions={
                    "succeeded": f"SAY_COMMUNICATE_OPERATORS",
                    "failed": f"ENABLE_MOVEBASE_MANAGER",
                },
            )

            smach.StateMachine.add(
                "ENABLE_MOVEBASE_MANAGER",
                smach_ros.ServiceState(
                "/pal_startup_control/start",
                StartupStart,
                request=StartupStartRequest("move_base", ""),
                ),
                transitions={
                    "succeeded": f"GO_TO_OPERATORS",
                    "preempted": "failed",
                    "aborted": "failed",
                },
            ),

            smach.StateMachine.add(
                "SAY_COMMUNICATE_OPERATORS",
                SayCommunicateOperators("ongoing"),
                transitions={
                    "succeeded": "GRASP_AND_PUT",
                    "aborted": "GRASP_AND_PUT",
                    "preempted": "GRASP_AND_PUT",
                },
            )

            smach.StateMachine.add(
                "SAY_GRASP_AND_PLACE",
                Say("ongoing"),
                transitions={
                    "succeeded": "SAY_FINISHED",
                    "aborted": "SAY_FINISHED",
                    "preempted": "SAY_FINISHED",
                },
            )

            # smach.StateMachine.add(
            #     "COMMUNICATE_OPERATORS",
            #     CommunicateOperators("No more object to put in cabinet"),
            #     transitions={
            #         "succeeded": "GRASP_AND_PUT",
            #         "aborted": "GRASP_AND_PUT",
            #         "preempted": "GRASP_AND_PUT",
            #     },
            # )

            # smach.StateMachine.add(
            #     "GRASP_AND_PUT",
            #     GraspAndPut(),
            #     transitions={
            #         "succeeded": "FIND_OPERATORS",
            #         "failed": "FIND_OPERATORS",
            #     },
            # )

            """
            Finish
            """

            smach.StateMachine.add(
                "SAY_FINISHED",
                Say(text="I am done."),
                transitions={
                    "succeeded": "succeeded",
                    "aborted": "failed",
                    "preempted": "succeeded",
                },
            )

def convert_yaml_to_pose(yaml_data):
    pose = Pose()
    yaml_data

    # Extract start_pose
    start_pose = yaml_data.get('arena', {}).get('start_pose', {})
    poses.append({
        'name': 'start_pose',
        'position': start_pose.get('position', {}),
        'orientation': start_pose.get('orientation', {})
    })

    # Iterate through rooms
    rooms = yaml_data.get('arena', {}).get('rooms', {})
    for room_name, room_data in rooms.items():
        # Room main pose
        room_pose = room_data.get('pose', {})
        poses.append({
            'name': f'{room_name}_pose',
            'position': room_pose.get('position', {}),
            'orientation': room_pose.get('orientation', {})
        })

        # People search poses
        for idx, psp in enumerate(room_data.get('people_search_poses', [])):
            poses.append({
                'name': f'{room_name}_people_search_pose_{idx}',
                'position': psp.get('position', {}),
                'orientation': psp.get('orientation', {})
            })

        # Beacons
        beacons = room_data.get('beacons', {})
        for beacon_name, beacon_data in beacons.items():
            pose = {
                'name': f'{room_name}_beacon_{beacon_name}',
                'position': {},  # beacons may not have a single "pose", but just polygon
                'orientation': {}
            }
            poses.append(pose)

    return poses
        