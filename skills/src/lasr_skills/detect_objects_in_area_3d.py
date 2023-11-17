#!/usr/bin/env python3

import smach

from lasr_skills import DetectObjects3D
from lasr_shapely import LasrShapely


class DetectObjectsInArea3D(smach.StateMachine):

    class FilterDetections(smach.State):

        def __init__(self):
            smach.State.__init__(self, outcomes=['succeeded', 'failed'], input_keys=['area_polygon','detections_3d'], output_keys=['detections_3d'])
            self.shapely = LasrShapely()

        def execute(self, userdata):
            satisfied_points = self.shapely.are_points_in_polygon_2d(userdata.area_polygon, [[pose[0], pose[1]] for (_, pose) in userdata.detections_3d]).inside
            filtered_detections = [userdata.detections_3d[i] for i in range(0, len(userdata.detections_3d)) if satisfied_points[i]]
            userdata.detections_3d = filtered_detections

            return 'succeeded'

    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'failed'], input_keys=['pcl_msg', 'area_polygon', 'filter'], output_keys=['detections_3d'])

        with self:

            smach.StateMachine.add('DETECT_OBJECTS_3D', DetectObjects3D(), transitions={'succeeded': 'FILTER_DETECTIONS', 'failed' : 'failed'})
            smach.StateMachine.add('FILTER_DETECTIONS', self.FilterDetections(), transitions={'succeeded' : 'succeeded', 'failed' : 'failed'})