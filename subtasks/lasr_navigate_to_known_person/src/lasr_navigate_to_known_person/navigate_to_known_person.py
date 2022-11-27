#!/usr/bin/env python3
import rospy
import smach

class ScanAndGoToHostSM():
    def __init__(self, base_controller, head_controller):
        self.s m =smach.StateMachine(outcomes=['succeeded', 'failed'], input_keys=['guest_list'], output_keys=['guest_list'])

        if rospy.has_param('/room'):
            pose = rospy.get_param('/room')
        else:
            pose = {
                'position' : {
                    'x' : 2.91,
                    'y' : -2.12,
                    'z' : 0
                },
                'orientation' : {
                    'x' : 0,
                    'y' : 0,
                    'z' : 0.7,
                    'w' : 0.7
                }
            }
        self.sm.userdata.location = Pose(Point(pose['position']['x'],
                                               pose['position']['y'],
                                               pose['position']['z']),
                                         Quaternion(pose['orientation']['x'],
                                                    pose['orientation']['y'],
                                                    pose['orientation']['z'],
                                                    pose['orientation']['w']))

        self.sm.userdata.prev = 'CollectInfoAndScan'

        self.base_controller = base_controller
        self.head_controller = head_controller

        with self.sm:
            self.sm.add('GO_TO_ROOM', GoTo(self.base_controller),
                        transitions={
                            'succeeded' : 'SCAN_ROOM',
                            'failed'    : 'failed'
                        },
                        remapping={
                            'location': 'location',
                            'prev' : 'prev'
                        })


            self.sm.add('SCAN_ROOM', Scan(self.head_controller),
                        transitions={
                            'succeeded' : 'GO_TO_HOST',
                            'failed'    : 'failed'
                        },
                        remapping={
                            'location': 'location',
                            'prev' : 'prev'
                        })


            self.sm.add('GO_TO_HOST', GoTo(self.base_controller),
                        transitions={
                            'succeeded' : 'succeeded',
                            'failed'    : 'failed'
                        },
                        remapping={
                            'location': 'location',
                            'prev' : 'prev'
                        })