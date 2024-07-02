from typing import List
import smach
import rospy
from lasr_vision_msgs.srv import (
    BodyPixKeypointDetection,
    BodyPixKeypointDetectionRequest,
)


# ALL_KEYS = {
#     'nose',
#     'leftEye',
#     'rightEye',
#     'leftEar',
#     'rightEar',
#     'leftShoulder',
#     'rightShoulder',
#     # 'leftElbow',
#     # 'rightElbow',
#     'leftWrist',
#     'rightWrist',
#     'leftHip',
#     'rightHip',
#     # 'leftKnee',
#     # 'rightKnee',
#     # 'leftAnkle',
#     # 'rightAnkle'
# }

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
    # 'nose',
    'leftEye',
    'rightEye',
    # 'leftEar',
    # 'rightEar',
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

ALL_KEYS_WITHOUT_TORSO = LEFT.union(RIGHT).union(HEAD).union(MIDDLE)

ALL_KEYS = ALL_KEYS_WITHOUT_TORSO.union(TORSO)


position_dict = {
    (3, -1): 'u3l',
    (3, 0): 'u3m',
    (3, 1): 'u3r',
    (2, -1): 'u2l',
    (2, 0): 'u2m',
    (2, 1): 'u2r',
    (1, -1): 'u1l',
    (1, 0): 'u1m',
    (1, 1): 'u1r',
    (0, -1): 'ml',
    (0, 0): 'mm',
    (0, 1): 'mr',
}


class AdjustCamera(smach.State):
    def __init__(
        self, 
        # keypoints_to_detect: List[str] = ALL_KEYS,
        bodypix_model: str = "resnet50",
        bodypix_confidence: float = 0.7,
        max_attempts=1000,
    ):
        smach.State.__init__(
            self,
            outcomes=["finished", "failed", 'u3l', 'u3m', 'u3r', 'u2l', 'u2m', 'u2r', 'u1l', 'u1m', 'u1r', 'ml', 'mm', 'mr',],
            input_keys=["img_msg", "position", "counter",],
            output_keys=["position", "counter",],
        )
        self.max_attempts = max_attempts
        # self._keypoints_to_detect = keypoints_to_detect
        self._bodypix_model = bodypix_model
        self._bodypix_confidence = bodypix_confidence
        self._bodypix_client = rospy.ServiceProxy(
            "/bodypix/keypoint_detection", BodyPixKeypointDetection
        )

        self.position = (2, 0)
        self.counter = 0
        
    def execute(self, userdata):
        req = BodyPixKeypointDetectionRequest()
        req.image_raw = userdata.img_msg
        req.dataset = self._bodypix_model
        req.confidence = self._bodypix_confidence
        req.keep_out_of_bounds = True

        try:
            res = self._bodypix_client(req)
        except Exception as e:
            print(e)
            return "failed"
        
        detected_keypoints = res.normalized_keypoints

        keypoint_names = [keypoint.keypoint_name for keypoint in detected_keypoints]
        rospy.logwarn(f"detected: {keypoint_names}")
        keypoint_info = {
            keypoint.keypoint_name: [keypoint.x, keypoint.y]
            for keypoint in detected_keypoints
        }
        # has_keypoints = {
        #     keypoint: keypoint in ALL_KEYS
        #     for keypoint in self._keypoints_to_detect
        #     }
        missing_keypoints = {
            keypoint
            for keypoint in ALL_KEYS
            if keypoint not in keypoint_names
        }
        try:
            position = [*userdata.position]
            counter = userdata.counter
        except KeyError:
            userdata.position = (2, 0)
            position = [*userdata.position]
            userdata.counter = 0
            counter = userdata.counter

        print(position)

        has_both_shoulders = len(missing_keypoints.intersection(MIDDLE)) == 0
        has_both_eyes = len(missing_keypoints.intersection(HEAD)) == 0

        has_more_than_one_shoulder = len(missing_keypoints.intersection(MIDDLE)) <= 1
        has_more_than_one_one_eye = len(missing_keypoints.intersection(HEAD)) <= 1

        rospy.logwarn(f"missing keypoints: {missing_keypoints}")
        rospy.logwarn(f"missing shoulders: {missing_keypoints.intersection(MIDDLE)}, missing eyes: {missing_keypoints.intersection(HEAD)}")
        # has_torso = len(missing_keypoints.intersection(TORSO)) <= 1
        
        if not has_more_than_one_shoulder and not has_more_than_one_one_eye:  
            # 'Try recovery behaviour or give up, need a bit polish
            miss_head = len(missing_keypoints.intersection(HEAD)) >= 2
            miss_middle = len(missing_keypoints.intersection(MIDDLE)) >= 2
            miss_torso = len(missing_keypoints.intersection(TORSO)) >= 4
            miss_left = len(missing_keypoints.intersection(LEFT)) >= 1
            miss_right = len(missing_keypoints.intersection(RIGHT)) >= 1
            rospy.logwarn(f"Attempts: {counter}, Missing head: {miss_head}, middle: {miss_middle}, torso: {miss_torso}, left: {miss_left}, right: {miss_right}.")
            needs_to_move_up = miss_head and (not miss_torso or not miss_middle)
            needs_to_move_down = not miss_head and miss_middle and miss_torso
            needs_to_move_left = miss_right
            needs_to_move_right = miss_left
            rospy.logwarn(f"Needs to move up: {needs_to_move_up}, down: {needs_to_move_down}, left: {needs_to_move_left}, right: {needs_to_move_right}.")
            
            # if counter > maxmum, check if head is in, if not, move up to get head, otherwise return finished.
            if counter > self.max_attempts:
                if not miss_head or counter > self.max_attempts + 2:
                    return "finished"
            
            counter += 1
            userdata.counter = counter
            if not (needs_to_move_left and needs_to_move_right):
                # return "failed"
                if needs_to_move_left:
                    userdata.position = (position[0], position[1] - 1 if position[1] > -1 else position[1])
                    return position_dict[userdata.position]
                if needs_to_move_right:
                    userdata.position = (position[0], position[1] + 1 if position[1] < 1 else position[1])          
                    return position_dict[userdata.position]
            if needs_to_move_up and needs_to_move_down:
                return "failed"
            if needs_to_move_up:
                userdata.position = (position[0] + 1 if position[0] < 3 else position[0], position[1])
                return position_dict[userdata.position]
            if needs_to_move_down:
                userdata.position = (position[0] - 1 if position[0] > 0 else position[0], position[1])
                return position_dict[userdata.position]
            return "finished"
        elif has_both_eyes and not has_both_shoulders:
            # in this case try to make eyes into the upper 1/3 of the frame,
            eyes_middle = ((keypoint_info["leftEye"][0] + keypoint_info["rightEye"][0]) / 2, (keypoint_info["leftEye"][1] + keypoint_info["rightEye"][1]) / 2)
            # if y at down 1/5: down move 2 steps
            if eyes_middle[1] >= 4/5:
                position[0] -= 2
            # if y at down 1/2: down move 1 step
            elif eyes_middle[1] >= 1/2:
                position[0] -= 1
            # if y at upper 1/3: wonder why no shoulders but never mind in this case
            else:
                pass
            # if x at left 1/3 or left shoulder dissappear, move left 1 step
            if eyes_middle[0] <= 1/3:
                position[1] -= 1
            # if x at right 1/3 or right shoulder dissappear, move right 1 step
            elif eyes_middle[0] >= 2/3:
                position[1] += 1
            pass
        elif not has_both_eyes and has_both_shoulders:
            shoulders_middle = ((keypoint_info["leftShoulder"][0] + keypoint_info["rightShoulder"][0]) / 2, (keypoint_info["leftEye"][1] + keypoint_info["rightEye"][1]) / 2)
            # if y at down 1/5: down move 1 step
            if shoulders_middle[1] >= 4/5:
                position[0] -= 1
            # if y at upper 1/4: up move 1 step
            elif shoulders_middle[1] <= 1/4:
                position[0] += 1
            # if x at left 1/3, move left 1 step
            if shoulders_middle[0] <= 1/3:
                position[1] -= 1
            # if x at right 1/3, move right 1 step
            elif shoulders_middle[0] >= 2/3:
                position[1] += 1
            pass
        elif has_both_eyes and has_both_shoulders:
            eyes_middle = ((keypoint_info["leftEye"][0] + keypoint_info["rightEye"][0]) / 2, (keypoint_info["leftEye"][1] + keypoint_info["rightEye"][1]) / 2)
            shoulders_middle = ((keypoint_info["leftShoulder"][0] + keypoint_info["rightShoulder"][0]) / 2, (keypoint_info["leftEye"][1] + keypoint_info["rightEye"][1]) / 2)
            very_middle = ((eyes_middle[0] + shoulders_middle[0]) / 2, (eyes_middle[1] + shoulders_middle[1]) / 2)
            rospy.logwarn(f"very middle {very_middle}")
            # if y at upper 1/5 for eyes: move up 1 step
            if eyes_middle[1] <= 1/5:
                position[0] += 1
                print('if y at upper 1/5 for eyes: move up 1 step')
            else:
                if 1/4 <= very_middle[1] <= 2/3 and 1/3 <= very_middle[0] <= 2/3:
                    print('finished.')
                    return "finished"
                # if y at down 1/3: down move 1 step
                if very_middle[1] >= 2/3:
                    position[0] -= 1
                    print('if y at down 1/3: down move 1 step.')
                # if y at upper 1/4: up move 1 step
                elif very_middle[1] <= 1/4:
                    position[0] += 1
                    print('if y at upper 1/3: up move 1 step.')
            # if x at left 1/3, move left 1 step
            if very_middle[0] <= 1/3:
                position[1] -= 1
                print('if x at left 1/3, move left 1 step.')
            # if x at right 1/3, move right 1 step
            elif very_middle[0] >= 2/3:
                position[1] += 1
                print('if x at right 1/3, move right 1 step.')
            pass
        elif has_more_than_one_shoulder:  # but not both
            # shoulders_middle = ((keypoint_info["leftShoulder"][0] + keypoint_info["rightShoulder"][0]) / 2, (keypoint_info["leftEye"][1] + keypoint_info["rightEye"][1]) / 2)
            # # move one step opposite left or right
            # # if x at left 1/3, move left 1 step
            # if shoulders_middle[0] <= 1/3:
            #     position[1] -= 1
            # # if x at right 1/3, move right 1 step
            # elif shoulders_middle[0] >= 2/3:
            #     position[1] += 1
            # pass
            # if not has_more_than_one_one_eye:
            #     # move up!
            #     position[0] += 1
            #     pass
            pass
        else:  # has_more_than_one_one_eye:
            # eyes_middle = ((keypoint_info["leftEye"][0] + keypoint_info["rightEye"][0]) / 2, (keypoint_info["leftEye"][1] + keypoint_info["rightEye"][1]) / 2)
            # # move one step opposite,
            # # if x at left 1/3, move left 1 step
            # if eyes_middle[0] <= 1/3:
            #     position[1] += 1
            # # if x at right 1/3, move right 1 step
            # elif eyes_middle[0] >= 2/3:
            #     position[1] -= 1
            # # probably move down
            # position[0] -= 1
            # pass
            pass

        if position[0] < 0:
            position[0] = 0
        elif position[0] > 3:
            position[0] = 3
        if position[1] < -1:
            position[1] = -1
        elif position[1] > 1:
            position[1] = 1
        
        userdata.position = (position[0], position[1])
        print(userdata.position)
        return position_dict[userdata.position]
    

# class AdjustCamera(smach.State):

#     def __init__(self, max_attempts=1000,):
#         smach.State.__init__(
#             self,
#             outcomes=["finished", "failed", 'u3l', 'u3m', 'u3r', 'u2l', 'u2m', 'u2r', 'u1l', 'u1m', 'u1r', 'ml', 'mm', 'mr',],
#             input_keys=["missing_keypoints", "position", "counter",],
#             output_keys=["position", "counter",],
#         )
#         self.max_attempts = max_attempts
        
#     def execute(self, userdata):
#         try:
#             position = userdata.position
#             counter = userdata.counter
#         except KeyError:
#             userdata.position = (2, 0)
#             position = userdata.position
#             userdata.counter = 0
#             counter = userdata.counter
#         print(userdata.position)
#         missing_keypoints = {key for key in userdata.missing_keypoints}
#         miss_head = len(missing_keypoints.intersection(HEAD)) >= 2
#         miss_middle = len(missing_keypoints.intersection(MIDDLE)) >= 2
#         miss_torso = len(missing_keypoints.intersection(TORSO)) >= 4
#         miss_left = len(missing_keypoints.intersection(LEFT)) >= 1
#         miss_right = len(missing_keypoints.intersection(RIGHT)) >= 1
#         rospy.logwarn(f"Attempts: {counter}, Missing head: {miss_head}, middle: {miss_middle}, torso: {miss_torso}, left: {miss_left}, right: {miss_right}.")
#         needs_to_move_up = miss_head and (not miss_torso or not miss_middle)
#         needs_to_move_down = not miss_head and miss_middle and miss_torso
#         needs_to_move_left = miss_right
#         needs_to_move_right = miss_left
#         rospy.logwarn(f"Needs to move up: {needs_to_move_up}, down: {needs_to_move_down}, left: {needs_to_move_left}, right: {needs_to_move_right}.")
        
#         # if counter > maxmum, check if head is in, if not, move up to get head, otherwise return finished.
#         if counter > self.max_attempts:
#             if not miss_head or counter > self.max_attempts + 2:
#                 return "finished"
        
#         counter += 1
#         userdata.counter = counter
#         if not (needs_to_move_left and needs_to_move_right):
#             # return "failed"
#             if needs_to_move_left:
#                 userdata.position = (position[0], position[1] - 1 if position[1] > -1 else position[1])
#                 return position_dict[userdata.position]
#             if needs_to_move_right:
#                 userdata.position = (position[0], position[1] + 1 if position[1] < 1 else position[1])            
#                 return position_dict[userdata.position]
#         if needs_to_move_up and needs_to_move_down:
#             return "failed"
#         if needs_to_move_up:
#             userdata.position = (position[0] + 1 if position[0] < 3 else position[0], position[1])
#             return position_dict[userdata.position]
#         if needs_to_move_down:
#             userdata.position = (position[0] - 1 if position[0] > 0 else position[0], position[1])
#             return position_dict[userdata.position]
#         return "finished"
