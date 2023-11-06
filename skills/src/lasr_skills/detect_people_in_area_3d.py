#!/usr/bin/env python3
import rospy
import smach

from lasr_skills import DetectPeople3D
from lasr_shapely import LasrShapely


class DetectPeopleInArea3D(smach.StateMachine):


    class FilterDetections(smach.State):

        def __init__(self):
            smach.State.__init__(self, outcomes=['succeeded', 'failed'], input_keys=['area_polygon','people_detections'], output_keys=['people_detections'])

            self.shapely = LasrShapely()

        def execute(self, userdata):
            satisfied_points = self.shapely.are_points_in_polygon_2d(userdata.area_polygon, [[pose[0], pose[1]] for (_, pose) in userdata.people_detections]).inside
            filtered_detections = [userdata.people_detections[i] for i in range(0, len(userdata.people_detections)) if satisfied_points[i]]
            userdata.people_detections = filtered_detections

            return 'succeeded'

    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'failed'], input_keys=['pcl_msg', 'area_polygon'], output_keys=['people_detections'])

        with self:

            smach.StateMachine.add('DETECT_PEOPLE_3D', DetectPeople3D(), transitions={'succeeded': 'FILTER_DETECTIONS', 'failed' : 'failed'})
            smach.StateMachine.add('FILTER_DETECTIONS', self.FilterDetections(), transitions={'succeeded' : 'succeeeded', 'failed' : 'failed'})