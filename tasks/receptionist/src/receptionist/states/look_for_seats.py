#!/usr/bin/env python3

import smach
import rospy
import numpy as np
from play_motion_msgs.msg import PlayMotionGoal
from geometry_msgs.msg import Point
from shapely.geometry import Polygon
from lasr_skills import DetectObjects3D, LookToPoint

class LookForSeats(smach.StateMachine):


    class Look(smach.State):

        def __init__(self, default):
            smach.State.__init__(self, outcomes=['done', 'not_done'], input_keys=['look_motion'])
            self.default = default
            self.motions = ['look_down_center']

        def execute(self, userdata):
            if not self.motions:
                return 'done'
            pm_goal = PlayMotionGoal(motion_name=self.motions.pop(0), skip_planning=True)
            self.default.pm.send_goal_and_wait(pm_goal)
            rospy.sleep(1.0)
            return 'not_done'

    class ProcessDetections(smach.State):

        def __init__(self):
            smach.State.__init__(self, outcomes=['succeeded'], input_keys=['detections_3d', 'bulk_people_detections_3d', 'bulk_seat_detections_3d'], output_keys=['bulk_people_detections_3d', 'bulk_seat_detections3d'])
        
        def execute(self, userdata):
            for det in userdata.detections_3d:
                if det[0].name == "person":
                    userdata.bulk_people_detections_3d.append(det)
                else:
                    userdata.bulk_seat_detections_3d.append(det)
            return 'succeeded'

    class CheckSeat(smach.State):
        def __init__(self, default):
            smach.State.__init__(self, outcomes=['not_done', 'succeeded', 'failed'],input_keys=['detections_3d', 'bulk_people_detections_3d', 'bulk_seat_detections_3d'], output_keys=['free_seat', 'bulk_people_detections_3d', 'bulk_seat_detections3d', 'point'])
            self.default = default

        def is_person_sitting_in_chair(self, person_contours, chair_contours):
            return Polygon(person_contours).intersects(Polygon(chair_contours))

        def execute(self, userdata):
            if not userdata.bulk_seat_detections_3d:
                return 'failed'
            det1, p1 = userdata.bulk_seat_detections_3d.pop(0)
            if not userdata.bulk_people_detections_3d:
                userdata.free_seat = [det1, p1]
                userdata.point = Point(*p1)
                print(f"Chair at {p1} is free!")
                return 'succeeded'
            for (det2, p2) in userdata.bulk_people_detections_3d:
                if not self.is_person_sitting_in_chair(np.array(det2.xyseg).reshape(-1, 2), np.array(det1.xyseg).reshape(-1, 2)):
                    userdata.free_seat = [det1, p1]
                    userdata.point = Point(*p1)
                    print(f"Chair at {p1} is free!")
                    self.default.voice.speak("Please sit down on this chair")
                    return 'succeeded'
            return 'not_done'

    def __init__(self, default):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'failed'])
        self.default = default
        self.userdata.filter = ["person", "chair"]
        self.userdata.bulk_people_detections_3d = []
        self.userdata.bulk_seat_detections_3d = []
        self.userdata.depth_topic = "xtion/depth_registered/points"

        with self:
            smach.StateMachine.add('LOOK', self.Look(self.default), transitions={'not_done' : 'DETECT_OBJECTS_3D', 'done' : 'CHECK_SEAT'})
            smach.StateMachine.add('DETECT_OBJECTS_3D', DetectObjects3D(), transitions={'succeeded' : 'PROCESS_DETECTIONS', 'failed' : 'failed'})
            smach.StateMachine.add('PROCESS_DETECTIONS', self.ProcessDetections(), transitions={'succeeded' : 'LOOK'})
            smach.StateMachine.add('CHECK_SEAT', self.CheckSeat(self.default), transitions={'succeeded' : 'FINALISE_SEAT', 'failed' : 'failed', 'not_done': 'CHECK_SEAT'})
            smach.StateMachine.add('FINALISE_SEAT', LookToPoint(), transitions={'succeeded' : 'succeeded'})

if __name__ == "__main__":
    from receptionist import Default
    rospy.init_node("test_look_for_seats")
    default = Default()
    sm = LookForSeats(default)
    sm.execute()