"""
State machine that introduces the greeted guest to all other guests/host present in the
seating area.

Ported from SMACH to YASMIN.
"""

import yasmin
import yasmin_ros
from shapely.geometry import Polygon as ShapelyPolygon

from lasr_skills import Say, DetectAllInPolygon, StartEyeTracker, StopEyeTracker

from HRI.states import ClearSeatingDetections, GetGuestData, GetIntroductionStr, Recognise


class Introduce(yasmin.StateMachine):
    """
    State machine that introduces a guest to all other guests/host present in
    the seating area.

    Replaces smach.Iterator with a CheckDone loop pattern.

    Blackboard keys required before calling sm():
        - guest_data: Dict of all guests keyed by id
        - guest_seat_point: PointStamped of the incoming guest's seat
        - seated_guest_locs: List of Point locations of all seated guests
        - person_index: Set to 0 before calling sm()
    """

    def __init__(self):
        super().__init__(outcomes=["succeeded", "failed"])
        self.add_input_key("guest_data")
        self.add_input_key("guest_seat_point")
        self.add_input_key("seated_guest_locs")
        
        self._node = yasmin_ros.logger_node
        
        self.seating_area = ShapelyPolygon(
            [
                self._node.get_parameter("seat_area.top_left").value,
                self._node.get_parameter("seat_area.top_right").value,
                self._node.get_parameter("seat_area.bottom_right").value,
                self._node.get_parameter("seat_area.bottom_left").value,
            ]
        )
        
        
        loop_state = yasmin.CbState(outcomes=['succeeded', 'continue'], callback=self._loop_person_index)
        loop_state.add_input_key('person_index')
        loop_state.add_input_key('people_detected')
        loop_state.add_input_key('guest_data')
        loop_state.add_output_key('person_index') 
        loop_state.add_output_key('person_point') 
        
        guest_loop = yasmin.CbState(outcomes=['succeeded', 'continue'], callback=self._loop_guest)
        guest_loop.add_input_key('guest_data')
        guest_loop.add_output_key('guest_data')
        
        
        self.add_state(
            'RESET_SEATING_DETECTIONS',
            ClearSeatingDetections(),
            transitions={'succeeded': 'FIND_PEOPLE', 'failed': 'failed'}
        )
        
        self.add_state(
            'FIND_PEOPLE',
            DetectAllInPolygon(
                polygon=self.seating_area,
                object_filter=['person'],
                min_coverage=1.0,
                min_new_object_dist=0.50,
                min_confidence=0.5,
            ),
            transitions = {'succeeded': 'LOOP_PERSON_STATE', 'failed': 'failed'},
            remappings={'detected_objects': 'people_detected'}
        )
        
        self.add_state(
            'LOOP_PERSON_STATE',
            loop_state,
            transitions={'succeeded': 'GRAB_GUEST_POINT', 'continue': 'LOOK_AT_PERSON'}
        )
        
        self.add_state(
            'LOOK_AT_PERSON',
            StartEyeTracker(),
            transitions={
                "succeeded": "RECOGNISE",
                "aborted": "failed",
                "canceled": "failed",
                "timeout": "RECOGNISE",
            }
        )
        
        self.add_state(
            'RECOGNISE',
            Recognise(),
            transitions={
                'succeeded': 'STOP_LOOK_AT_PERSON',
                'aborted': 'failed',
                'no_detections': 'STOP_LOOK_AT_PERSON'
            }
        )
        
        self.add_state(
            'STOP_LOOK_AT_PERSON',
            StopEyeTracker(),
            transitions={
                "succeeded": "LOOP_PERSON_STATE",
                "aborted": "failed",
                "canceled": "failed",
                "timeout": "failed",
            },
        )
        
        self.add_state(
            'GRAB_GUEST_POINT',
            guest_loop,
            transitions={'succeeded': 'succeeded', 'continue': 'START_EYE_TRACKING_GUEST'}
        )
        
        self.add_state(
            'START_EYE_TRACKING_GUEST',
            StartEyeTracker(),
            transitions={
                "succeeded": "RECOGNISE",
                "aborted": "failed",
                "canceled": "failed",
                "timeout": "RECOGNISE",
            },
            remappings={'person_point': 'guest_point'}
        )
        
        self.add_state(
            'GET_INTRODUCTION_STR',
            GetIntroductionStr(),
            transitions={
                'succeeded': 'SAY_INTRODUCTION',
                'failed': 'failed'
            }
        )
        
        self.add_state(
            'SAY_INTRODUCTION',
            Say(),
            transitions={
                "succeeded": "STOP_EYE_TRACKING",
                "aborted": "STOP_EYE_TRACKING",
                "canceled": "STOP_EYE_TRACKING",
            },
        )
        
        self.add_state(
            'STOP_EYE_TRACKING',
            StopEyeTracker(),
            transitions={
                "succeeded": "GRAB_GUEST_POINT",
                "aborted": "failed",
                "canceled": "failed",
                "timeout": "failed",
            },
        )

    def _loop_person_index(self, blackboard):
        if blackboard['guest_data']['guest1']['seated_point'] is not None and blackboard['guest_data']['guest2']['seated_point'] is not None:
            return 'succeeded'
        elif blackboard['person_index'] is None:
            blackboard['person_index'] = 0
        elif blackboard['person_index'] < len(blackboard['people_detected']) - 1:
            blackboard['person_index'] += 1
        else:
            return 'succeeded'
        
        index = blackboard['person_index']
        blackboard['person_point'] = blackboard['people_detected'][index].point
        return 'continue'
    
    def _loop_guest(self, blackboard):
        if blackboard['guest_data']['guest1']['seating_detection'] and blackboard['guest_data']['guest2']['seating_detection']:
            return 'succeeded'
        
        id = 'guest1' if not blackboard['guest_data']['guest1']['seating_detection'] else 'guest2'
        
        blackboard['guest_point'] = blackboard['guest_data'][id]['seated_point'] 
        blackboard['guest_data'][id]['seating_detection'] = True
        blackboard['introduce_to'] = id
        blackboard['relevant_guest_data'] = blackboard['guest_data']['guest2'] if id == 'guest1' else blackboard['guest_data']['guest1']
        return 'continue'