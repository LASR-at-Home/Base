import smach
import rospy


class AdjustCamera(smach.State):

    def __init__(self,):
        smach.State.__init__(
            self,
            outcomes=["finished"],
            input_keys=["missing_keypoints"],
            output_keys=[],
        )
        
    def execute(self, userdata):
        missing_keypoints = userdata.missing_keypoints
        print(missing_keypoints)
