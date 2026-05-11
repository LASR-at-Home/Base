from typing import List, Tuple, Dict

import rclpy
import smach
import smach_ros
from geometry_msgs.msg import Point, PointStamped, Pose

from lasr_skills import Say, GoToLocation

from HRI.states import *

from shapely.geometry import Polygon
from std_msgs.msg import Empty


class HRI(smach.StateMachine):
    def __init__(self,
                 node,
                 host_data,
                 face_detection_confidence = 0.2):
        super().__init__(outcomes=["succeeded", "failed"])
        
        def wait_cb(ud, msg):
            return False
        
        with self:
            self.userdata.guest_data = {
                "host": host_data,
                "guest1": {
                    "name": "",
                    "drink": "",
                    "detection": False,
                    "seating_detection": False,
                },
                "guest2": {
                    "name": "",
                    "drink": "",
                    "detection": False,
                    "seating_detection": False,
                },
            }
            drink_detections = {
            }

            self.userdata.drink_detections = drink_detections
            self.userdata.confidence = face_detection_confidence
            self.userdata.dataset = "receptionist"
            self.userdata.drink_position = PointStamped()
        
            
            self.add('WAIT_START',  # Awaits start Signal for the task
                     smach_ros.MonitorState(node=node, topic='/receptionist/start', msg_type=Empty, cond_cb=wait_cb, max_checks=10),
                     transitions={'invalid': 'START_TIMER', 'valid': 'WAIT_START', 'preempted': 'WAIT_START'})
            
            self.add('START_TIMER',
                     StartTimer(node=node),
                     transitions={'succeeded': 'START_CON', 'failed': 'START_TIMER'})
        
            self.add('START_CON',   # SM1: Waits for Door to open, then goes to start
                    self.setup(node=node),
                    transitions={"succeeded": "GREET", "failed": "GREET"})
            
            self.add('GREET',       # SM2: Greets guest
                     LookAndGreetGuest(node=node, last_resort=False, guest_id='guest1'),
                     transitions={'succeeded': 'GUIDE_TO_SEAT', 'failed': 'failed'})
            
            self.add('GUIDE_TO_SEAT', # GUIDES GUEST TO SEATING AREA
                     GoToLocation(node=node, location_param="seat_pose"),
                     transitions={'succeeded': 'SEAT_GUEST', 'failed': 'failed'})
            
            self.add('SEAT_GUEST', # SM3: Locates and seats guest in free seat
                     SeatGuest(node=node, learn_host=False),
                     transitions={'succeeded': 'succeeded', 'failed': 'failed'}) 
        
        
        # commented incase Detect Doorbell was not implemented
        # smach.StateMachine.add(
        #     "DETECT DOORBELL",
        #     DetectDoorbell(node),
        #     transitions={
        #         "valid": "APPROACH_GUEST",
        #         "invalid": "DETECT_DOORBELL",
        #         "preempted": "DETECT_DOORBELL",
        #     },
        # )

        # start door state machine goes here (by Fadi and Aldrich)

        # smach.StateMachine.add(
        #     "APPROACH_GUEST",
        #     ApproachGuest(node),
        #     transitions={
        #         "valid": "FACE_GUEST",
        #         "invalid": "GREET_GUEST",
        #         "preempted": "GREET_GUEST",
        #     },
        # )

        # # face guest and greet them concurrently state machine (by Fadi)

        # smach.StateMachine.add(
        #     "GET_NAME_AND_DRINK",
        #     GetNameAndDrink(node),
        #     transitions={
        #         "valid": "GUIDE_GUEST_TO_LIVING_ROOM",
        #         "invalid": "GET_NAME_AND_DRINK",
        #         "preempted": "GET_NAME_AND_DRINK",
        #     },
        # )

        # smach.StateMachine.add(
        #     "GUIDE_GUEST_TO_LIVING_ROOM",
        #     GuideGuestsToLivingroom(node),
        #     transitions={
        #         "valid": "OFFER_A_FREE_SEAT",
        #         "invalid": "GUIDE_GUEST_TO_LIVING_ROOM",
        #         "preempted": "GUIDE_GUEST_TO_LIVING_ROOM",
        #     },
        # )

        # smach.StateMachine.add(
        #     "CHECK_SOFA",
        #     CheckSofa(node),
        #     transitions={
        #         "valid": "OFFER_A_FREE_SEAT",
        #         "invalid": "CHECK_SOFA",
        #         "preempted": "CHECK_SOFA",
        #     },
        # )

        # smach.StateMachine.add(
        #     "OFFER_A_FREE_SEAT",
        #     OfferFreeSeat(node),
        #     transitions={
        #         "valid": "INTRODUCE_GUESTS_TO_EACHOTHER",
        #         "invalid": "OFFER_A_FREE_SEAT",
        #         "preempted": "OFFER_A_FREE_SEAT",
        #     },
        # )

        # smach.StateMachine.add(
        #     "INTRODUCE_GUESTS_TO_EACHOTHER",
        #     IntroduceGuestsToEachother(node),
        #     transitions={
        #         "valid": "ASK_SECOND_GUEST_FOR_BAG_TO_HOST",
        #         "invalid": "INTRODUCE_GUESTS_TO_EACHOTHER",
        #         "preempted": "INTRODUCE_GUESTS_TO_EACHOTHER",
        #     },
        # )

        # smach.StateMachine.add(
        #     "ASK_SECOND_GUEST_FOR_BAG_TO_HOST",
        #     AskSecondGuestForBagToHost(node),
        #     transitions={
        #         "valid": "PICK_UP_BAG",
        #         "invalid": "ASK_SECOND_GUEST_FOR_BAG_TO_HOST",
        #         "preempted": "ASK_SECOND_GUEST_FOR_BAG_TO_HOST",
        #     },
        # )

        # smach.StateMachine.add(
        #     "PICK_UP_BAG",
        #     PickUpBag(node),
        #     transitions={
        #         "valid": "FOLLOW_HOST",
        #         "invalid": "LISTEN_TO_HOST_DROP_INSTRUCTION",
        #         "preempted": "LISTEN_TO_HOST_DROP_INSTRUCTION",
        #     },
        # )

        # smach.StateMachine.add(
        #     "LISTEN_TO_HOST_DROP_BAG_INSTRUCTION",
        #     ListenToHostDropBagInstruction(node),
        #     transitions={
        #         "valid": "DROP_BAG",
        #         "invalid": "LISTEN_TO_HOST_DROP_BAG_INSTRUCTION",
        #         "preempted": "LISTEN_TO_HOST_DROP_BAG_INSTRUCTION",
        #     },
        # )

        # smach.StateMachine.add(
        #     "DROP_BAG",
        #     DropBag(node),
        #     transitions={
        #         "valid": "succeeded",
        #         "invalid": "DROP_BAG",
        #         "preempted": "DROP_BAG",
        #     },
        # )


    def setup(self, node):
        start_con_sm = smach.Concurrence(
                outcomes=["succeeded", "failed"],
                default_outcome="failed",
                outcome_map={
                    "succeeded": {
                        "SAY_START": "succeeded",
                        "DOOR_START": "succeeded",
                    },
                    "failed": {
                        "SAY_START": "aborted",
                        "DOOR_START": "failed",
                    },
                },
            )
            
        with start_con_sm:
            smach.Concurrence.add(
                "SAY_START", Say(node=node, text="Start of HRI task.")
            )

            smach.Concurrence.add(
                "DOOR_START", StartDoorSM(node=node)
            )
            
        return start_con_sm
    
    
def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node(
        node_name="hri",
        allow_undeclared_parameters=True,
        automatically_declare_parameters_from_overrides=True,
    )

    sm = HRI(node=node, host_data={})
    outcome = sm.execute()
    node.get_logger().info(f"StartSM outcome: {outcome}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()