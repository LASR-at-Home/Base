#!/usr/bin/env python3
import rospy
import smach

from lasr_skills import DetectPeopleInArea3D

from sensor_msgs.msg import PointCloud2

class WaitForPersonInArea(smach.StateMachine):

    class GetPointCloud(smach.State):

        def __init__(self):
            smach.State.__init__(self, outcomes=['succeeded'], output_keys=['pcl_msg'])
        
        def execute(self, userdata):
            userdata.pcl_msg = rospy.wait_for_message("/xtion/depth_registered/points", PointCloud2)
            return 'succeeded'

    class CheckForPerson(smach.State):

        def __init__(self):
            smach.State.__init__(self, outcomes=['done', 'not_done'], input_keys=['people_detections_3d'])

        def execute(self, userdata):
            if len(userdata.people_detections_3d):
                return 'done'
            else:
                return 'not_done'
            
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'failed'], input_keys=['area_polygon'], output_keys=['people_detections_3d'])


        with self:
            smach.StateMachine.add('GET_POINTCLOUD', self.GetPointCloud(), transitions={'succeeded' : 'DETECT_PEOPLE_3D'})
            smach.StateMachine.add('DETECT_PEOPLE_3D', DetectPeopleInArea3D(), transitions={'succeeded' : 'CHECK_FOR_PERSON', 'failed' : 'failed'})
            smach.StateMachine.add('CHECK_FOR_PERSON', self.CheckForPerson(), transitions={'done' : 'succeeded', 'not_done' : 'GET_POINTCLOUD'})

if __name__ == "__main__":
    rospy.init_node("test_wait_for_person")
    sm = smach.StateMachine(outcomes=['end'])

    class MockA(smach.State):

        def __init__(self):
            smach.State.__init__(self, outcomes=['1'], output_keys=['area_polygon'])

        def execute(self, userdata):
            userdata.area_polygon = [[1.9, 2.8], [3.17, 2.95], [1.69, 4.01], [2.88, 4.26]]
            return '1'

    class MockC(smach.State):

        def __init__(self):
            smach.State.__init__(self, outcomes=['1'], input_keys=['people_detections_3d'])

        def execute(self, userdata):
            print(userdata.people_detections_3d)
            return '1'

    with sm:
        sm.add('A', MockA(), transitions={'1' : 'WAIT_FOR_PERSON'})
        sm.add('WAIT_FOR_PERSON', WaitForPersonInArea(), transitions={'succeeded' : 'C', 'failed' : 'end'})
        sm.add('C', MockC(), transitions={'1' : 'end'})

    sm.execute()
