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
    # 'leftEar',
    'leftShoulder',
}

RIGHT = {
    'rightEye',
    # 'rightEar',
    'rightShoulder',
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


position_dict = {
    (2, -1): 'u2l',
    (2, 0): 'u2m',
    (2, 1): 'u2r',
    (1, -1): 'u1l',
    (1, 0): 'u1m',
    (1, 1): 'u1r',
    (0, -1): 'dl',
    (0, 0): 'mm',
    (0, 1): 'mr',
}


class AdjustCamera(smach.State):

    def __init__(self, max_attempts=3,):
        smach.State.__init__(
            self,
            outcomes=["finished", "failed", 'u2l', 'u2m', 'u2r', 'u1l', 'u1m', 'u1r', 'ml', 'mm', 'mr',],
            input_keys=["missing_keypoints", "position", "counter",],
            output_keys=["position", "counter",],
        )
        self.max_attempts = max_attempts
        
    def execute(self, userdata):
        try:
            position = userdata.position
            counter = userdata.counter
        except KeyError:
            userdata.position = (2, 0)
            position = userdata.position
            userdata.counter = 0
            counter = userdata.counter
        print(userdata.position)
        missing_keypoints = {key for key in userdata.missing_keypoints}
        miss_head = len(missing_keypoints.intersection(HEAD)) >= 3
        miss_middle = len(missing_keypoints.intersection(MIDDLE)) >= 2
        miss_torso = len(missing_keypoints.intersection(TORSO)) >= 4
        miss_left = len(missing_keypoints.intersection(LEFT)) >= 2
        miss_right = len(missing_keypoints.intersection(RIGHT)) >= 2
        rospy.logwarn(f"Attempts: {counter}, Missing head: {miss_head}, middle: {miss_middle}, torso: {miss_torso}, left: {miss_left}, right: {miss_right}.")
        needs_to_move_up = miss_head and (not miss_torso or not miss_middle)
        needs_to_move_down = not miss_head and miss_torso
        needs_to_move_left = miss_right
        needs_to_move_right = miss_left
        rospy.logwarn(f"Needs to move up: {needs_to_move_up}, down: {needs_to_move_down}, left: {needs_to_move_left}, right: {needs_to_move_right}.")
        
        # if counter > maxmum, check if head is in, if not, move up to get head, otherwise return finished.
        if counter > self.max_attempts:
            if needs_to_move_up:
                userdata.position = (position[0] + 1 if position[0] < 2 else position[0], position[1])
                return position_dict[userdata.position] 
            else:
                return "finished"
        
        counter += 1
        userdata.counter = counter
        if needs_to_move_up and needs_to_move_down:
            return "failed"
        if needs_to_move_up:
            userdata.position = (position[0] + 1 if position[0] < 2 else position[0], position[1])
            return position_dict[userdata.position]
        if needs_to_move_down:
            userdata.position = (position[0] - 1 if position[0] > 0 else position[0], position[1])
            return position_dict[userdata.position]
        if needs_to_move_left and needs_to_move_right:
            return "failed"
        if needs_to_move_left:
            userdata.position = (position[0], position[1] - 1 if position[1] > -1 else position[1])
            return position_dict[userdata.position]
        if needs_to_move_right:
            userdata.position = (position[0], position[1] + 1 if position[1] < 1 else position[1])            
            return position_dict[userdata.position]
        return "finished"
