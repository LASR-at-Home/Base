#!/usr/bin/env python3
import rospy
import smach

from lasr_skills import DetectPeople

from sensor_msgs.msg import Image

class WaitForPerson(smach.StateMachine):

    class GetImage(smach.State):

        def __init__(self):
            smach.State.__init__(self, outcomes=['succeeded'], output_keys=['img_msg'])
        
        def execute(self, userdata):
            userdata.img_msg = rospy.wait_for_message("/xtion/rgb/image_raw", Image)
            return 'succeeded'

    class CheckForPerson(smach.State):

        def __init__(self):
            smach.State.__init__(self, outcomes=['done', 'not_done'], input_keys=['detections'])

        def execute(self, userdata):
            if len(userdata.detections.detected_objects):
                return 'done'
            else:
                return 'not_done'
            
    def __init__(self):

        smach.StateMachine.__init__(self, outcomes=['succeeded', 'failed'], output_keys=['detections'])

        with self:
            smach.StateMachine.add('GET_IMAGE', self.GetImage(), transitions={'succeeded' : 'DETECT_PEOPLE'})
            smach.StateMachine.add('DETECT_PEOPLE', DetectPeople(), transitions={'succeeded' : 'CHECK_FOR_PERSON', 'failed' : 'failed'})
            smach.StateMachine.add('CHECK_FOR_PERSON', self.CheckForPerson(), transitions={'done' : 'succeeded', 'not_done' : 'GET_IMAGE'})