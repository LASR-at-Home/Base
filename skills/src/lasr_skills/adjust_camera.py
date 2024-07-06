import smach
import rospy
from lasr_vision_msgs.srv import (
    BodyPixKeypointDetection,
    BodyPixKeypointDetectionRequest,
)
from .vision import GetCroppedImage
from lasr_skills.play_motion import PlayMotion
import rospkg
import rosparam
import os


LEFT = {
    "leftEye",
    # 'leftEar',
    "leftShoulder",
}

RIGHT = {
    "rightEye",
    # 'rightEar',
    "rightShoulder",
}

HEAD = {
    # 'nose',
    "leftEye",
    "rightEye",
    # 'leftEar',
    # 'rightEar',
}

MIDDLE = {
    "leftShoulder",
    "rightShoulder",
}

TORSO = {
    "leftWrist",
    "rightWrist",
    "leftHip",
    "rightHip",
}

ALL_KEYS_WITHOUT_TORSO = LEFT.union(RIGHT).union(HEAD).union(MIDDLE)

ALL_KEYS = ALL_KEYS_WITHOUT_TORSO.union(TORSO)

positions = [
    "u3l",
    "u3m",
    "u3r",
    "u2l",
    "u2m",
    "u2r",
    "u1l",
    "u1m",
    "u1r",
    "ml",
    "mm",
    "mr",
]

position_dict = {
    (3, -1): "u3l",
    (3, 0): "u3m",
    (3, 1): "u3r",
    (2, -1): "u2l",
    (2, 0): "u2m",
    (2, 1): "u2r",
    (1, -1): "u1l",
    (1, 0): "u1m",
    (1, 1): "u1r",
    (0, -1): "ml",
    (0, 0): "mm",
    (0, 1): "mr",
}

inverse_position_dict = {value: key for key, value in position_dict.items()}


class AdjustCamera(smach.StateMachine):
    def __init__(
        self,
        bodypix_model: str = "resnet50",
        bodypix_confidence: float = 0.7,
        max_attempts=5,
        debug=False,
        init_state="u1m",
    ):
        smach.StateMachine.__init__(
            self,
            outcomes=[
                "finished",
                "failed",
                "truncated",
            ],
            input_keys=[],
            output_keys=[],
        )
        r = rospkg.RosPack()
        els = rosparam.load_file(
            os.path.join(r.get_path("lasr_skills"), "config", "motions.yaml")
        )
        for param, ns in els:
            rosparam.upload_params(ns, param)

        with self:
            smach.StateMachine.add(
                "init",
                PlayMotion(motion_name=init_state),
                transitions={
                    "succeeded": "GET_IMAGE",
                    "aborted": "GET_IMAGE",
                    "preempted": "GET_IMAGE",
                },
            )

            if debug:
                _transitions = {
                    "succeeded": "DECIDE_ADJUST_CAMERA",
                    "failed": "GET_IMAGE",
                }
            else:
                _transitions = {
                    "succeeded": "DECIDE_ADJUST_CAMERA",
                    "failed": "GET_IMAGE",
                }
            smach.StateMachine.add(
                "GET_IMAGE",
                GetCroppedImage(
                    object_name="person",
                    method="closest",
                    use_mask=True,
                ),
                transitions=_transitions,
            )

            if debug:
                _transitions = {
                    "finished": "GET_IMAGE",
                    "failed": "GET_IMAGE",
                    "truncated": "GET_IMAGE",
                }
            else:
                _transitions = {
                    "finished": "finished",
                    "failed": "failed",
                    "truncated": "truncated",
                }
            for position in positions:
                _transitions[position] = position

            smach.StateMachine.add(
                "DECIDE_ADJUST_CAMERA",
                self.DecideAdjustCamera(
                    bodypix_model=bodypix_model,
                    bodypix_confidence=bodypix_confidence,
                    max_attempts=max_attempts,
                    init_state=init_state,
                ),
                transitions=_transitions,
            )

            for motion in positions:
                smach.StateMachine.add(
                    motion,
                    PlayMotion(motion_name=motion),
                    transitions={
                        "succeeded": "GET_IMAGE",
                        "aborted": "GET_IMAGE",
                        "preempted": "GET_IMAGE",
                    },
                )

    class DecideAdjustCamera(smach.State):
        def __init__(
            self,
            bodypix_model: str = "resnet50",
            bodypix_confidence: float = 0.7,
            max_attempts=1000,
            init_state="u1m",
        ):
            smach.State.__init__(
                self,
                outcomes=[
                    "finished",
                    "failed",
                    "truncated",
                ]
                + positions,
                input_keys=[
                    "img_msg",
                ],
                output_keys=[],
            )
            self.max_attempts = max_attempts
            self._bodypix_model = bodypix_model
            self._bodypix_confidence = bodypix_confidence
            self._bodypix_client = rospy.ServiceProxy(
                "/bodypix/keypoint_detection", BodyPixKeypointDetection
            )

            self.position = [i for i in inverse_position_dict[init_state]]
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
            missing_keypoints = {
                keypoint for keypoint in ALL_KEYS if keypoint not in keypoint_names
            }

            has_both_shoulders = len(missing_keypoints.intersection(MIDDLE)) == 0
            has_both_eyes = len(missing_keypoints.intersection(HEAD)) == 0

            has_more_than_one_shoulder = (
                len(missing_keypoints.intersection(MIDDLE)) <= 1
            )
            has_more_than_one_one_eye = len(missing_keypoints.intersection(HEAD)) <= 1

            rospy.logwarn(f"missing keypoints: {missing_keypoints}")
            rospy.logwarn(
                f"missing shoulders: {missing_keypoints.intersection(MIDDLE)}, missing eyes: {missing_keypoints.intersection(HEAD)}"
            )

            if not has_more_than_one_shoulder and not has_more_than_one_one_eye:
                # This is the case that not any centre points can be used,
                # In this case most likely it is the guest standing either too close or not in the camera at all.
                # However we may still try to get this person back into the frame if some part of them are detected.
                # Otherwise we say something like "Please stand in front of me but keep a bit distance.".
                rospy.logwarn(
                    "The person might not actually be in the frame, trying to recover."
                )
                miss_head = len(missing_keypoints.intersection(HEAD)) >= 2
                miss_middle = len(missing_keypoints.intersection(MIDDLE)) >= 2
                miss_torso = len(missing_keypoints.intersection(TORSO)) >= 4
                miss_left = len(missing_keypoints.intersection(LEFT)) >= 1
                miss_right = len(missing_keypoints.intersection(RIGHT)) >= 1
                rospy.logwarn(
                    f"Missing head: {miss_head}, middle: {miss_middle}, torso: {miss_torso}, left: {miss_left}, right: {miss_right}."
                )
                needs_to_move_up = miss_head and (not miss_torso or not miss_middle)
                needs_to_move_down = not miss_head and miss_middle and miss_torso
                needs_to_move_left = miss_right
                needs_to_move_right = miss_left
                rospy.logwarn(
                    f"Needs to move up: {needs_to_move_up}, down: {needs_to_move_down}, left: {needs_to_move_left}, right: {needs_to_move_right}."
                )

                if not (needs_to_move_left and needs_to_move_right):
                    if needs_to_move_left:
                        self.position = (
                            self.position[0],
                            (
                                self.position[1] - 1
                                if self.position[1] > -1
                                else self.position[1]
                            ),
                        )
                    elif needs_to_move_right:
                        self.position = (
                            self.position[0],
                            (
                                self.position[1] + 1
                                if self.position[1] < 1
                                else self.position[1]
                            ),
                        )
                if not needs_to_move_up and needs_to_move_down:
                    if needs_to_move_up:
                        self.position = (
                            (
                                self.position[0] + 1
                                if self.position[0] < 3
                                else self.position[0]
                            ),
                            self.position[1],
                        )
                    elif needs_to_move_down:
                        self.position = (
                            (
                                self.position[0] - 1
                                if self.position[0] > 0
                                else self.position[0]
                            ),
                            self.position[1],
                        )

            elif has_both_eyes and not has_both_shoulders:
                # in this case try to make eyes into the upper 1/3 of the frame,
                eyes_middle = (
                    (keypoint_info["leftEye"][0] + keypoint_info["rightEye"][0]) / 2,
                    (keypoint_info["leftEye"][1] + keypoint_info["rightEye"][1]) / 2,
                )
                # if y at down 1/5: down move 2 steps
                if eyes_middle[1] >= 4 / 5:
                    self.position[0] -= 2
                # if y at down 1/2: down move 1 step
                elif eyes_middle[1] >= 1 / 2:
                    self.position[0] -= 1
                # if y at upper 1/3: wonder why no shoulders but never mind in this case
                else:
                    pass
                # if x at left 2/7 or left shoulder dissappear, move left 1 step
                if eyes_middle[0] <= 2 / 7:
                    self.position[1] -= 1
                # if x at right 2/7 or right shoulder dissappear, move right 1 step
                elif eyes_middle[0] >= 5 / 7:
                    self.position[1] += 1
                pass

            elif not has_both_eyes and has_both_shoulders:
                shoulders_middle = (
                    (
                        keypoint_info["leftShoulder"][0]
                        + keypoint_info["rightShoulder"][0]
                    )
                    / 2,
                    (keypoint_info["leftEye"][1] + keypoint_info["rightEye"][1]) / 2,
                )
                # if y at down 1/5: down move 1 step
                if shoulders_middle[1] >= 4 / 5:
                    self.position[0] -= 1
                # if y at upper 1/4: up move 1 step
                elif shoulders_middle[1] <= 1 / 4:
                    self.position[0] += 1
                # if x at left 2/7, move left 1 step
                if shoulders_middle[0] <= 2 / 7:
                    self.position[1] -= 1
                # if x at right 2/7, move right 1 step
                elif shoulders_middle[0] >= 5 / 7:
                    self.position[1] += 1
                pass

            elif has_both_eyes and has_both_shoulders:
                eyes_middle = (
                    (keypoint_info["leftEye"][0] + keypoint_info["rightEye"][0]) / 2,
                    (keypoint_info["leftEye"][1] + keypoint_info["rightEye"][1]) / 2,
                )
                shoulders_middle = (
                    (
                        keypoint_info["leftShoulder"][0]
                        + keypoint_info["rightShoulder"][0]
                    )
                    / 2,
                    (keypoint_info["leftEye"][1] + keypoint_info["rightEye"][1]) / 2,
                )
                very_middle = (
                    (eyes_middle[0] + shoulders_middle[0]) / 2,
                    (eyes_middle[1] + shoulders_middle[1]) / 2,
                )
                rospy.logwarn(f"very middle {very_middle}")
                # if y at upper 1/5 for eyes: move up 1 step
                if eyes_middle[1] <= 1 / 5:
                    self.position[0] += 1
                    print("if y at upper 1/5 for eyes: move up 1 step")
                else:
                    if (
                        1 / 4 <= very_middle[1] <= 2 / 3
                        and 1 / 3 <= very_middle[0] <= 2 / 3
                    ):
                        print("finished.")
                        return "finished"
                    # if y at down 1/3: down move 1 step
                    if very_middle[1] >= 2 / 3:
                        self.position[0] -= 1
                        print("if y at down 1/3: down move 1 step.")
                    # if y at upper 1/4: up move 1 step
                    elif very_middle[1] <= 1 / 4:
                        self.position[0] += 1
                        print("if y at upper 1/3: up move 1 step.")
                # if x at left 2/7, move left 1 step
                if very_middle[0] <= 2 / 7:
                    self.position[1] -= 1
                    print("if x at left 2/7, move left 1 step.")
                # if x at right 2/7, move right 1 step
                elif very_middle[0] >= 5 / 7:
                    self.position[1] += 1
                    print("if x at right 2/7, move right 1 step.")
                pass

            # keep the position in the range.
            if self.position[0] < 0:
                self.position[0] = 0
            elif self.position[0] > 3:
                self.position[0] = 3
            if self.position[1] < -1:
                self.position[1] = -1
            elif self.position[1] > 1:
                self.position[1] = 1

            # if counter > maxmum.
            if self.counter > self.max_attempts:
                return "truncated"
            self.counter += 1

            return position_dict[(self.position[0], self.position[1])]
