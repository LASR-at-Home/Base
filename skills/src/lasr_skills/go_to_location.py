#!/usr/bin/env python3

import rospy
import smach
from tiago_controllers.controllers import Controllers
from geometry_msgs.msg import Point, Quaternion, Pose

class GoToLocation(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'], input_keys=['location'])
        self.controllers = Controllers()

    def execute(self, userdata):
        try:
            status = self.controllers.base_controller.sync_to_pose(userdata.location)
            if status:
                return 'succeeded'
            return 'failed'
        except rospy.ERROR as e:
            rospy.logwarn(f"Unable to go to location. {userdata.location} -> ({str(e)})")
            return 'failed'

if __name__ == '__main__':
    rospy.init_node('go_to_location')
    sm = smach.StateMachine(outcomes=['succeeded', 'failed'])
    loc = rospy.get_param('/living_room/table/location')
    sm.userdata.location = Pose(position=Point(**loc['position']), orientation=Quaternion(**loc['orientation']))
    with sm:
        smach.StateMachine.add('GoToLocation', GoToLocation(), transitions={'succeeded': 'succeeded', 'failed': 'failed'})
    sm.execute()