#!/usr/bin/env python3
import rospy
import smach

from lasr_skills import DetectPeople3D
from lasr_shapely import LasrShapely

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
            smach.State.__init__(self, outcomes=['done', 'not_done'], input_keys=['area_polygon', 'people_detections'], output_keys=['filtered_people_detections'])

        def execute(self, userdata):
            satisfied_points = self.shapely.are_points_in_polygon_2d(userdata.area_polygon, [[pose[0], pose[1]] for (_, pose) in userdata.people_detections]).inside
            filtered_detections = [userdata.people_detections[i] for i in range(0, len(userdata.people_detections)) if satisfied_points[i]]
            userdata.filtered_people_detections = filtered_detections
            if len(filtered_detections):
                return 'done'
            else:
                return 'not done'
            
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'failed'], input_keys=['area_polygon'], output_keys=['people_detections'])

        self.shapely = LasrShapely()

        with self:
            smach.StateMachine.add('GET_POINTCLOUD', self.GetPointCloud(), transitions={'succeeded' : 'DETECT_PEOPLE_3D'})
            smach.StateMachine.add('DETECT_PEOPLE_3D', DetectPeople3D(), transitions={'succeeded' : 'CHECK_FOR_PERSON', 'failed' : 'failed'})
            smach.StateMachine.add('CHECK_FOR_PERSON', self.CheckForPerson(), transitions={'done' : 'succeeded', 'not_done' : 'GET_POINTCLOUD'})

if __name__ == "__main__":
    rospy.init_node("test_wait_for_person")
    sm = smach.StateMachine(outcomes=['end'])

    class MockC(smach.State):

        def __init__(self):
            smach.State.__init__(self, outcomes=['1'], input_keys=['people_detections'])

        def execute(self, userdata):
            print(userdata.filtered_people_detections)
            return '1'

    with sm:
        sm.add('WAIT_FOR_PERSON', WaitForPersonInArea(), transitions={'succeeded' : 'C', 'failed' : 'end'})
        sm.add('C', MockC(), transitions={'1' : 'end'})

    sm.execute()
