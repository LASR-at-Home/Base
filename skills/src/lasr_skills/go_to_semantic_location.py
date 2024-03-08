#!/usr/bin/env python3

import rospy
import smach
from tiago_controllers.controllers import Controllers
from geometry_msgs.msg import Point, Quaternion, Pose

class GoToSemanticLocation(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'], input_keys=['location'])
        self.controllers = Controllers()

    def execute(self, userdata):
        loc = rospy.get_param(f"{userdata.location}/location")
        try:
            status = self.controllers.base_controller.sync_to_pose(Pose(position=Point(**loc['position']), orientation=Quaternion(**loc['orientation'])))
            if status:
                return 'succeeded'
            return 'failed'
        except rospy.ERROR as e:
            rospy.logwarn(f"Unable to go to location. {loc} -> ({str(e)})")
            return 'failed'
if __name__ == '__main__':
    rospy.init_node('go_to_semantic_location')
    sm = smach.StateMachine(outcomes=['succeeded', 'failed'])
    sm.userdata.location = '/living_room/table'
    with sm:
        smach.StateMachine.add('GoToSemanticLocation', GoToSemanticLocation(), transitions={'succeeded': 'succeeded', 'failed': 'failed'})
    sm.execute()
