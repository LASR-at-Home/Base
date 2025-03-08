import rclpy
import smach
from lasr_vision_interfaces.srv import BodyPixKeypointDetection
from .vision import GetCroppedImage
from lasr_skills.play_motion import PlayMotion
from lasr_skills import AccessNode

LEFT = {"leftEye", "leftShoulder"}
RIGHT = {"rightEye", "rightShoulder"}
HEAD = {"leftEye", "rightEye"}
MIDDLE = {"leftShoulder", "rightShoulder"}
TORSO = {"leftWrist", "rightWrist", "leftHip", "rightHip"}
ALL_KEYS = LEFT.union(RIGHT).union(HEAD).union(MIDDLE).union(TORSO)

positions = [
    "u3l", "u3m", "u3r",
    "u2l", "u2m", "u2r",
    "u1l", "u1m", "u1r",
    "ml", "mm", "mr",
]

position_dict = {
    (3, -1): "u3l", (3, 0): "u3m", (3, 1): "u3r",
    (2, -1): "u2l", (2, 0): "u2m", (2, 1): "u2r",
    (1, -1): "u1l", (1, 0): "u1m", (1, 1): "u1r",
    (0, -1): "ml", (0, 0): "mm", (0, 1): "mr",
}

inverse_position_dict = {value: key for key, value in position_dict.items()}


class AdjustCamera(smach.StateMachine):
    def __init__(
        self, bodypix_confidence=0.7, max_attempts=5, debug=False, init_state="u1m"
    ):
        smach.StateMachine.__init__(
            self, outcomes=["finished", "failed", "truncated"], input_keys=[], output_keys=[]
        )
        self.node = AccessNode.get_node()

        self.bodypix_client = self.create_client(BodyPixKeypointDetection, "/bodypix/keypoint_detection")
        while not self.bodypix_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for BodyPixKeypointDetection service...")

        self.position = [i for i in inverse_position_dict[init_state]]
        self.max_attempts = max_attempts
        self.bodypix_confidence = bodypix_confidence
        self.counter = 0

        with self:
            smach.StateMachine.add(
                "INIT", PlayMotion(motion_name=init_state),
                transitions={"succeeded": "GET_IMAGE", "aborted": "GET_IMAGE", "preempted": "GET_IMAGE"},
            )

            smach.StateMachine.add(
                "GET_IMAGE",
                GetCroppedImage(object_name="person", method="closest", use_mask=True),
                transitions={"succeeded": "DECIDE_ADJUST_CAMERA", "failed": "GET_IMAGE"},
            )

            smach.StateMachine.add(
                "DECIDE_ADJUST_CAMERA",
                self.DecideAdjustCamera(self),
                transitions={pos: pos for pos in positions}.update(
                    {"finished": "finished", "failed": "failed", "truncated": "truncated"}
                ),
            )

            for motion in positions:
                smach.StateMachine.add(
                    motion, PlayMotion(motion_name=motion),
                    transitions={"succeeded": "GET_IMAGE", "aborted": "GET_IMAGE", "preempted": "GET_IMAGE"},
                )

    class DecideAdjustCamera(smach.State):
        def __init__(self,):
            smach.State.__init__(
                self,
                outcomes=["finished", "failed", "truncated"] + positions,
                input_keys=["img_msg"],
                output_keys=[],
            )
            self.node = AccessNode.get_node()
            self.counter = 0

        def execute(self, userdata):
            self.node.get_logger().warn(f"Start attempt number {self.counter}.")

            req = BodyPixKeypointDetection.Request()
            req.image_raw = userdata.img_msg
            req.confidence = self.node.bodypix_confidence
            req.keep_out_of_bounds = True

            future = self.node.bodypix_client.call_async(req)
            rclpy.spin_until_future_complete(self.node, future)
            try:
                res = future.result()
            except Exception as e:
                self.node.get_logger().error(f"Service call failed: {e}")
                return "failed"

            detected_keypoints = res.normalized_keypoints
            keypoint_names = [kp.keypoint_name for kp in detected_keypoints]
            self.node.get_logger().warn(f"Detected: {keypoint_names}")

            missing_keypoints = {kp for kp in ALL_KEYS if kp not in keypoint_names}
            self.node.get_logger().warn(f"Missing keypoints: {missing_keypoints}")

            has_both_shoulders = len(missing_keypoints.intersection(MIDDLE)) == 0
            has_both_eyes = len(missing_keypoints.intersection(HEAD)) == 0

            if not has_both_shoulders and not has_both_eyes:
                self.node.get_logger().warn("Person might not be in frame, trying to recover.")

            if self.counter > self.node.max_attempts:
                return "truncated"
            self.counter += 1
            return position_dict.get(tuple(self.node.position), "failed")
