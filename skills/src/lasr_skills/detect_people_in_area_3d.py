#!/usr/bin/env python3
import rospy
import smach

from lasr_skills import DetectPeople3D
from lasr_shapely import LasrShapely


class DetectPeopleInArea3D(smach.StateMachine):


    class FilterDetections(smach.State):

        def __init__(self):
            smach.State.__init__(self, outcomes=['succeeded', 'failed'], input_keys=['area_polygon','people_detections_3d'], output_keys=['people_detections_3d'])

            self.shapely = LasrShapely()

        def execute(self, userdata):
            satisfied_points = self.shapely.are_points_in_polygon_2d(userdata.area_polygon, [[pose[0], pose[1]] for (_, pose) in userdata.people_detections_3d]).inside
            filtered_detections = [userdata.people_detections_3d[i] for i in range(0, len(userdata.people_detections_3d)) if satisfied_points[i]]
            userdata.people_detections_3d = filtered_detections

            return 'succeeded'

    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'failed'], input_keys=['pcl_msg', 'area_polygon'], output_keys=['people_detections_3d'])

        with self:

            smach.StateMachine.add('DETECT_PEOPLE_3D', DetectPeople3D(), transitions={'succeeded': 'FILTER_DETECTIONS', 'failed' : 'failed'})
            smach.StateMachine.add('FILTER_DETECTIONS', self.FilterDetections(), transitions={'succeeded' : 'succeeded', 'failed' : 'failed'})

if __name__ == "__main__":
    rospy.init_node("test_detect_people_in_area_3d")
    sm = smach.StateMachine(outcomes=['end'])

    class MockA(smach.State):

        def __init__(self):
            smach.State.__init__(self, outcomes=['1'], output_keys=['pcl_msg', 'area_polygon'])

        def execute(self, userdata):
            from sensor_msgs.msg import PointCloud2
            pcl_msg = rospy.wait_for_message("/xtion/depth_registered/points", PointCloud2)
            userdata.pcl_msg = pcl_msg
            userdata.area_polygon = [[1.9, 2.8], [3.17, 2.95], [1.69, 4.01], [2.88, 4.26]]
            return '1'

    class MockC(smach.State):

        def __init__(self):
            smach.State.__init__(self, outcomes=['1'], input_keys=['people_detections_3d'])

        def execute(self, userdata):
            print(userdata.people_detections_3d)
            return '1'

    with sm:
        sm.add('A', MockA(), transitions={'1' : 'DETECT_PEOPLE_3D'})
        sm.add('DETECT_PEOPLE_3D', DetectPeopleInArea3D(), transitions={'succeeded' : 'C', 'failed' : 'end'})
        sm.add('C', MockC(), transitions={'1' : 'end'})

    sm.execute()
