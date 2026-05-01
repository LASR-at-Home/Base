import threading
from typing import Union
from geometry_msgs.msg import Pose, Point, Quaternion

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from smach import StateMachine

from .states import go_to_location, detect_3d, detect_faces, detect_3d_in_area, face_person, say, get_name_and_drink, start_eye_tracker, listen, speech_recovery

"""
Robot is already at door, since:
    SM1 detects door ---(door opening)---> robot goes through door to waiting area ---(waits a few seconds)---> robot goes back to door area
    
So Robot is at door and it:
    Faces person -> Start Eye tracker -> Greet guest -> Listen -> recognise name and drink -> listen loop
"""

# TODO: Add 

class ApproachGuest(StateMachine):
    def __init__(self, 
                 node):
        super().__init__(outcomes=['succeeded', 'failed'], input_keys=['guest_data'], output_keys=['guest_data'])
        
        with self:
            super().add(
                'LOOK_AT_PERSON',
                face_person(node=node),
                transitions={'finished': 'START_EYE_TRACKING', 'failed': 'LOOK_AT_PERSON', 'truncated': 'failed'}
            )
            super().add(
                'START_EYE_TRACKING',
                start_eye_tracker(node=node),
                transitions={'succeeded': 'GREET_GUEST', 'aborted': 'START_EYE_TRACKING', 'preempted': 'failed'}
            )
            super().add(
                'GREET_GUEST',
                say(node=node, text="Hello there"),
                transitions={'succeeded': 'LISTEN', 'aborted': 'LOOK_AT_PERSON', 'preempted': 'failed'}
            )
            super().add(
                'LISTEN',
                listen(node=node),
                transitions={'succeeded': 'RECOGNISE_NAME_DRINK', 'aborted': 'APOLOGISE', 'preempted': 'failed'},
                remapping={'sequence': 'guest_transcription'}
            )
            super().add(
                'APOLOGISE',
                say(node=node, text="Sorry, I didn't quite catch that"),
                transitions={'succeeded': 'LISTEN', 'aborted': 'failed', 'preempted': 'failed'}
            )
            super().add(
                'RECOGNISE_NAME_DRINK',
                get_name_and_drink(), #REFACTOR STATE
                transitions={}
            )