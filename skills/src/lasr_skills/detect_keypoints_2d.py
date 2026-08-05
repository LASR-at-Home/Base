#!/usr/bin/env python3
import time
from collections import deque

import rclpy
import yasmin
import yasmin_ros
import numpy as np
from yasmin import Blackboard, StateMachine
from yasmin_ros import set_ros_loggers, ServiceState


from sensor_msgs.msg import Image
from lasr_vision_interfaces.srv import YoloPoseDetection


# ==========================================
# 1. 2D 키포인트 추출 (Service Client)
# ==========================================
class DetectKeypoints2D(ServiceState):
    def __init__(
        self,
        image_topic: str = "/image_raw",
        model: str = "yolo11n-pose.pt",
        confidence: float = 0.5,
    ):
        super().__init__(
            srv_type=YoloPoseDetection,
            srv_name="/yolo/detect_pose",
            create_request_handler=self._create_req,
            outcomes=["succeeded", "failed", "aborted"],
            response_handler=self.response_handler,
        )

        self.add_output_key("keypoint_detections")
        self.model = model
        self.confidence = confidence
        self.node = yasmin_ros.logger_node
        self.image_msg = None

        self.sub = self.node.create_subscription(
            Image, image_topic, self.image_callback, 10
        )

    def image_callback(self, msg):
        self.image_msg = msg

    def execute(self, blackboard):
        if self.image_msg is None:
            self.node.get_logger().warn("Waiting for webcam image on /image_raw...")
            time.sleep(1.0)
            return "aborted"
        return super().execute(blackboard)

    def _create_req(self, blackboard):
        req = YoloPoseDetection.Request()
        req.image_raw = self.image_msg
        req.model = self.model
        req.confidence = self.confidence
        return req

    def response_handler(self, blackboard, response):
        if len(response.detections) == 0:
            return "failed"
        blackboard["keypoint_detections"] = response
        return "succeeded"


# ==========================================
# 2. 2D 픽셀 기반 자세 판정
# ==========================================
class EvaluatePosture2D(yasmin.State):
    def __init__(self):
        super().__init__(outcomes=["succeeded", "failed"])
        self.history = deque(maxlen=15)
        # 유연성을 위해 파라미터로 설정
        self.waving_threshold = 0.3  # 어깨너비의 몇 % 이상 움직여야 흔드는지
        self.sitting_threshold = 0.5  # 어깨너비 대비 골반-무릎 높이 비율

    def get_angle(self, a, b, c):
        # a, b, c는 YOLO 결과의 객체이므로 .x, .y 속성을 가짐
        ba = np.array([a.x - b.x, a.y - b.y])
        bc = np.array([c.x - b.x, c.y - b.y])

        # 0 나누기 방지
        norm_ba = np.linalg.norm(ba)
        norm_bc = np.linalg.norm(bc)
        if norm_ba == 0 or norm_bc == 0:
            return 0.0

        cosine_angle = np.dot(ba, bc) / (norm_ba * norm_bc)
        return np.degrees(np.arccos(np.clip(cosine_angle, -1.0, 1.0)))


class EvaluatePosture2D(yasmin.State):
    def __init__(self):
        super().__init__(outcomes=["succeeded", "failed"])
        self.r_history = deque(maxlen=15)
        self.l_history = deque(maxlen=15)

    # [수정] 클래스 내부 메서드로 정의하여 NameError 해결
    def is_waving(self, wrist, shoulder, history, shoulder_width):
        if wrist.y < shoulder.y:
            history.append(wrist.x)
            if len(history) == 15:
                return (max(history) - min(history)) > (shoulder_width * 0.3)
        else:
            history.clear()
        return False

    def execute(self, blackboard):
        if "keypoint_detections" not in blackboard:
            return "failed"

        person = blackboard["keypoint_detections"].detections[0]
        kp = {k.keypoint_name: k for k in person.keypoints}

        if not all(
            k in kp
            for k in [
                "right_shoulder",
                "left_shoulder",
                "right_wrist",
                "left_wrist",
                "right_hip",
                "right_knee",
            ]
        ):
            return "failed"

        shoulder_width = abs(kp["right_shoulder"].x - kp["left_shoulder"].x) + 1e-6

        candidates = {
            "waving": False,
            "Pointing": False,
            "Raising Arm": False,
            "Sitting": False,
        }
        pointing_dir = "None"

        # 1. Waving 판정 (self.is_waving으로 호출)
        if self.is_waving(
            kp["right_wrist"], kp["right_shoulder"], self.r_history, shoulder_width
        ) or self.is_waving(
            kp["left_wrist"], kp["left_shoulder"], self.l_history, shoulder_width
        ):
            candidates["waving"] = True

        # 2. Pointing 판정
        if not candidates["waving"]:
            is_arm_horizontal = abs(kp["right_wrist"].y - kp["right_shoulder"].y) < (
                shoulder_width * 0.7
            )
            if is_arm_horizontal:
                if kp["right_wrist"].x > kp["right_shoulder"].x + (
                    shoulder_width * 0.2
                ):
                    candidates["Pointing"] = True
                    pointing_dir = "Left"
                elif kp["right_wrist"].x < kp["right_shoulder"].x - (
                    shoulder_width * 0.2
                ):
                    candidates["Pointing"] = True
                    pointing_dir = "Right"

        # 3. Raising Arm
        if not candidates["waving"] and not candidates["Pointing"]:
            # 오른쪽 어깨보다 손목이 높은지 확인 (사람 기준 오른쪽)
            right_arm_raised = kp["right_wrist"].y < kp["right_shoulder"].y
            # 왼쪽 어깨보다 손목이 높은지 확인 (사람 기준 왼쪽)
            left_arm_raised = kp["left_wrist"].y < kp["left_shoulder"].y

            if right_arm_raised or left_arm_raised:
                candidates["Raising Arm"] = True
                if right_arm_raised and left_arm_raised:
                    arm_dir = "Both"
                elif right_arm_raised:
                    arm_dir = "Right"
                else:
                    arm_dir = "Left"

        # 4. Sitting
        if abs(kp["right_hip"].y - kp["right_knee"].y) < (shoulder_width * 0.5):
            candidates["Sitting"] = True

        # 최종 판정 시 출력 수정

        # 최종 판정
        if candidates["waving"]:
            detected_posture = "waving"
        elif candidates["Pointing"]:
            detected_posture = f"Pointing {pointing_dir}"
        elif candidates["Raising Arm"]:
            detected_posture = f"Raising Arm {arm_dir}"  # 방향 포함!
        elif candidates["Sitting"]:
            detected_posture = "Sitting"
        else:
            detected_posture = "Standing"

        yasmin.YASMIN_LOG_INFO(f"=== FINAL POSTURE: {detected_posture} ===")
        blackboard["pointing_direction"] = pointing_dir
        return "succeeded"


# ==========================================
# 3. State Machine 실행 (Main)
# ==========================================
def main():
    rclpy.init()
    set_ros_loggers()

    yasmin.YASMIN_LOG_INFO("Starting 2D Human Posture Recognition SM...")
    sm = StateMachine(outcomes=["succeeded", "failed", "aborted"], handle_sigint=True)
    bb = Blackboard()

    sm.add_state(
        "DETECT2D_POSE",
        DetectKeypoints2D(),
        transitions={
            "succeeded": "EVALUATE_POSTURE",
            "failed": "DETECT2D_POSE",
            "aborted": "DETECT2D_POSE",
        },
    )

    sm.add_state(
        "EVALUATE_POSTURE",
        EvaluatePosture2D(),
        transitions={"succeeded": "DETECT2D_POSE", "failed": "DETECT2D_POSE"},
    )

    outcome = sm(bb)
    yasmin.YASMIN_LOG_INFO(f"State Machine Finished with: {outcome}")

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
