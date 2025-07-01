# ~/your_ws/src/lasr_skills/src/lasr_skills/fake_nav.py
import smach
import rospy
from geometry_msgs.msg import Pose

class GoToLocation(smach.State):
    """
    A fake GoToLocation that immediately returns success.
    """
    def __init__(self, location: Pose = None):
        super().__init__(outcomes=["succeeded","failed"])
        self.location = location

    def execute(self, userdata):
        rospy.loginfo(f"[FAKE_NAV] skipping move to {self.location}")
        return "succeeded"
