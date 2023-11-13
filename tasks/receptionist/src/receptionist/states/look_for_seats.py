#!/usr/bin/env python3

import smach
from play_motion_msgs.msg import PlayMotionGoal

from lasr_skills import DetectObjects3D

class LookForSeats(smach.StateMachine):


    class Look(smach.State):

        def __init__(self, default):
            smach.State.__init__(self, outcomes=['done', 'not_done'], input_keys=['look_motion'])
            self.default = default
            self.motions = ['look_left', 'look_right', 'look_center']

        def execute(self, userdata):
            if not self.motions:
                return 'done'
            pm_goal = PlayMotionGoal(motion_name=self.motions.pop(0), skip_planning=True)
            self.default.pm.send_goal_and_wait(pm_goal)
            return 'not_done'

    class ProcessDetections(smach.State):

        def __init__(self):
            smach.State.__init__(self, outcomes=['succeeded'], input_keys=['detections_3d', 'bulk_detections_3d'], output_keys=['bulk_detections_3d'])
        
        def execute(self, userdata):
            userdata.bulk_detections_3d.extend(userdata.detections_3d)
            print(userdata.bulk_detections_3d)
            return 'succeeded'

    def __init__(self, default):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'failed'])
        self.default = default
        self.userdata.filter = ["person", "chair"]
        self.userdata.bulk_detections_3d = []
        self.userdata.depth_topic = "xtion/depth_registered/points"

        with self:
            smach.StateMachine.add('LOOK', self.Look(self.default), transitions={'not_done' : 'DETECT_OBJECTS_3D', 'done' : 'succeeded'})
            smach.StateMachine.add('DETECT_OBJECTS_3D', DetectObjects3D(), transitions={'succeeded' : 'PROCESS_DETECTIONS', 'failed' : 'failed'})
            smach.StateMachine.add('PROCESS_DETECTIONS', self.ProcessDetections(), transitions={'succeeded' : 'LOOK'})

if __name__ == "__main__":
    import rospy
    from receptionist import Default
    rospy.init_node("test_look_for_seats")
    default = Default()
    sm = LookForSeats(default)
    sm.execute()