"""
State for parsing the transcription of the guests' name and favourite interest, and adding this
to the guest data userdata
"""

import rospy
import smach
from smach import UserData
from typing import List, Dict, Any
from receptionist.states import SpeechRecovery
from lasr_llm_msgs.srv import Llm, LlmRequest
import string
from geometry_msgs.msg import (
    Pose,
    Point,
    Quaternion,
    Polygon,
    PoseWithCovarianceStamped,
    PointStamped,
)


class HandleRequestLLM(smach.StateMachine):
    def __init__(
        self, last_resort: bool, param_key: str = "/give_me_a_hand/priors"
    ):

        self._param_key = param_key
        self._last_resort = last_resort

        smach.StateMachine.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=[],
            output_keys=["location"],
        )
        with self:
            smach.StateMachine.add(
                "PARSE_REQUEST",
                self.ParseRequest(
                    last_resort=self._last_resort,
                    param_key=self._param_key,
                ),
                transitions={"succeeded":"succeeded","failed":"RECOVER_REQUEST"}
            )


    class ParseRequest(smach.State):
        def __init__(
            self,
            last_resort: bool,
            param_key: str = "/give_me_a_hand/priors",
        ):
            """Parses the transcription of the guests' name and interest.

            Args:
                param_key (str, optional): Name of the parameter that contains the list of
                prior knowledge . Defaults to "/receptionist/priors".
            """
            smach.State.__init__(
                self,
                outcomes=["succeeded", "failed"],
                input_keys=["guest_transcription", "guest_data"],
                output_keys=["guest_data", "guest_transcription"],
            )
            self._llm = rospy.ServiceProxy("/lasr_llm/llm", Llm)
            self._llm.wait_for_service()
            prior_data: Dict[str, List[str]] = rospy.get_param(param_key)
            self._possible_names = [name.lower() for name in prior_data["names"]]
            self._last_resort = last_resort

        def execute(self, userdata: UserData) -> str:

            #Do getting location pose from LLM business 

            location_string = "kitchen_table"
            location_pose = self.get_location_pose(location_string,False,False)
            userdata.location = location_pose
            return "succeeded"


                
        def get_location_room(self,location: str) -> str:
            rooms = rospy.get_param("/gpsr/arena/rooms")
            for room in rooms:
                if location in rooms[room]["beacons"]:
                    return room
            raise ValueError(f"Location {location} not found in the arena")

        def get_location_pose(self,location: str, person: bool, dem_manipulation=False) -> Pose:
            location_room = self.get_location_room(location)
            if person:
                location_pose: Dict = rospy.get_param(
                    f"/gpsr/arena/rooms/{location_room}/beacons/{location}/person_detection_pose"
                )
            elif not dem_manipulation:
                location_pose: Dict = rospy.get_param(
                    f"/gpsr/arena/rooms/{location_room}/beacons/{location}/object_detection_pose"
                )
            else:
                location_pose: Dict = rospy.get_param(
                    f"/gpsr/arena/rooms/{location_room}/beacons/{location}/dem_manipulation_pose"
                )

            return Pose(
                position=Point(**location_pose["position"]),
                orientation=Quaternion(**location_pose["orientation"]),
            )

        
