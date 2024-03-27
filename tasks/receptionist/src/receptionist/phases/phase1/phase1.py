import smach
from .states import Start
from .states import GoToWaitForPerson
from .states import GoToPerson
from lasr_skills import WaitForPersonInArea
from lasr_skils import GoToLocation
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion


def get_pose_from_param(name):
    if not rospy.has_param(name):
        return None
    pose = rospy.get_param(name)
    print(pose)
    return Pose(Point(pose['position']['x'],
                    pose['position']['y'],
                    pose['position']['z']),
                Quaternion(pose['orientation']['x'],
                        pose['orientation']['y'],
                        pose['orientation']['z'],
                        pose['orientation']['w']))

def get_pose_from_param_new(name, col='location'):
    if not rospy.has_param(name):
        return None
    pose = rospy.get_param(name)
    position, orientation = pose[col]["position"], pose[col]["orientation"]
    return Pose(position=Point(**position), orientation=Quaternion(**orientation))

class Phase1(smach.StateMachine):
    def __init__(self, default):
        smach.StateMachine.__init__(self, outcomes=['success'])

        with self:
            smach.StateMachine.add('START', Start(self.default), transitions={'succeeded' : 'GO_TO_WAIT_FOR_PERSON'})
            smach.StateMachine.add('GO_TO_WAIT_FOR_PERSON',GoToLocation(),transitions={'aborted' : 'GO_TO_WAIT_FOR_PERSON','succeeded': 'WAIT_FOR_PERSON'},remapping={"location":"wait_location"})
            smach.StateMachine.add('WAIT_FOR_PERSON', WaitForPersonInArea([[1.94, 0.15], [2.98, 0.28], [3.08, -0.68], [2.06, -0.84]]) ,transitions={'failed' : 'failed','succeeded':'GO_TO_PERSON'})
            smach.StateMachine.add('GO_TO_PERSON', GoToLocation(),transitions={'succeeded':'succeeded'},remapping={'location':'person_location'})


            