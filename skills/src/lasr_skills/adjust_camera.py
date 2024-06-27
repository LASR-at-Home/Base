import smach
import rospy


ALL_KEYS = {
    'nose',
    'leftEye',
    'rightEye',
    'leftEar',
    'rightEar',
    'leftShoulder',
    'rightShoulder',
    # 'leftElbow',
    # 'rightElbow',
    'leftWrist',
    'rightWrist',
    'leftHip',
    'rightHip',
    # 'leftKnee',
    # 'rightKnee',
    # 'leftAnkle',
    # 'rightAnkle'
}

LEFT = {
    'leftEye',
    'leftEar',
    'leftShoulder',
    'leftElbow',
    'leftWrist',
    'leftHip',
    'leftKnee',
    'leftAnkle',
}

RIGHT = {
    'rightEye',
    'rightEar',
    'rightShoulder',
    'rightElbow',
    'rightWrist',
    'rightHip',
    'rightKnee',
    'rightAnkle',
}

HEAD = {
    'nose',
    'leftEye',
    'rightEye',
    'leftEar',
    'rightEar',
}

MIDDLE = {
    'leftShoulder',
    'rightShoulder',
}

TORSO = {
    'leftWrist',
    'rightWrist',
    'leftHip',
    'rightHip',
}


class AdjustCamera(smach.State):

    def __init__(self,):
        smach.State.__init__(
            self,
            outcomes=["finished"],
            input_keys=["missing_keypoints"],
            output_keys=[],
        )
        
    def execute(self, userdata):
        missing_keypoints = {key for key in userdata.missing_keypoints}
        miss_head = len(missing_keypoints.intersection(HEAD)) >= 3
        miss_middle = len(missing_keypoints.intersection(MIDDLE)) >= 2
        miss_torso = len(missing_keypoints.intersection(TORSO)) >= 3
        miss_left = len(missing_keypoints.intersection(LEFT)) >= 5
        miss_right = len(missing_keypoints.intersection(RIGHT)) >= 5
        rospy.logwarn(f"Missing head: {miss_head}, middle: {miss_middle}, torso: {miss_torso}, left: {miss_left}, right: {miss_right}.")
        return "finished"
