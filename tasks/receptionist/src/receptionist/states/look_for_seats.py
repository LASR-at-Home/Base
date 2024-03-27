#!/usr/bin/env python3

import smach
import rospy
import numpy as np
import math
from play_motion_msgs.msg import PlayMotionGoal
from geometry_msgs.msg import Point
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
from lasr_skills import Detect3D
from copy import copy

class LookForSeats(smach.StateMachine):


    class Look(smach.State):

        def __init__(self, default):
            smach.State.__init__(self, outcomes=['done', 'not_done'], input_keys=['look_motion'])
            self.default = default
            self.motions = ['look_down_left', 'look_down_center', 'look_down_right']
            self.remaining_motions = copy(self.motions)

        def execute(self, userdata):
            if not self.remaining_motions:
                self.remaining_motions = copy(self.motions)
                return 'done'
            pm_goal = PlayMotionGoal(motion_name=self.remaining_motions.pop(0), skip_planning=True)
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
                robot_x, robot_y, _ = self.default.controllers.base_controller.get_current_pose()
                r = np.array([robot_x, robot_y])
                p = np.array([p1[0], p1[1]])
                theta = np.degrees(np.arccos(np.dot(r, p) / (np.linalg.norm(r) * np.linalg.norm(p))))
                print(theta)
                if theta > 10.0:
                    self.default.voice.sync_tts("Please sit down on this chair to my right")
                elif theta < -10.0:
                    self.default.voice.sync_tts("Please sit down on this chair to my left")
                else:
                    self.default.voice.sync_tts("Please sit down on this chair")
                    self.remaining_motions = []
                return 'succeeded'
            for (det2, _) in userdata.bulk_people_detections_3d:
                if not self.is_person_sitting_in_chair(np.array(det2.xyseg).reshape(-1, 2), np.array(det1.xyseg).reshape(-1, 2)):
                    userdata.free_seat = [det1, p1]
                    userdata.point = Point(*p1)
                    print(f"Chair at {p1} is free!")
                    robot_x, robot_y, _ = self.default.controllers.base_controller.get_current_pose()
                    r = np.array([robot_x, robot_y])
                    p = np.array([p1[0], p1[1]])
                    theta = np.degrees(np.arccos(np.dot(r, p) / (np.linalg.norm(r) * np.linalg.norm(p))))
                    print(theta)
                    if theta > 10.0:
                        self.default.voice.sync_tts("Please sit down on this chair to my right")
                    elif theta < -10.0:
                        self.default.voice.sync_tts("Please sit down on this chair to my left")
                    else:
                        self.default.voice.sync_tts("Please sit down on this chair")
                    self.remaining_motions = []
                    return 'succeeded'
            return 'not_done'

    class PointToChair(smach.State):
        
        def __init__(self, default):
            smach.State.__init__(self, outcomes=['succeeded'], input_keys=['point'])
            self.default = default
        
        def execute(self, userdata):

           # pm_goal = PlayMotionGoal(motion_name="back_to_default_head", skip_planning=True)
            #self.default.pm.send_goal_and_wait(pm_goal)

#            self.default.controllers.base_controller.sync_face_to(userdata.point.x, userdata.point.y)

 #           pm_goal = PlayMotionGoal(motion_name="raise_torso", skip_planning=True)
  #          self.default.pm.send_goal_and_wait(pm_goal)
            
   #         pm_goal = PlayMotionGoal(motion_name="point", skip_planning=False)
    #        self.default.pm.send_goal_and_wait(pm_goal)

     #       rospy.sleep(5.0)

      #      pm_goal = PlayMotionGoal(motion_name="home", skip_planning=False)
       #     self.default.pm.send_goal_and_wait(pm_goal)

            return 'succeeded'


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
            smach.StateMachine.add('FINALISE_SEAT', self.PointToChair(self.default), transitions={'succeeded' : 'succeeded'})

if __name__ == "__main__":
    from receptionist import Default
    rospy.init_node("test_look_for_seats")
    default = Default()
    sm = LookForSeats(default)
    sm.execute()