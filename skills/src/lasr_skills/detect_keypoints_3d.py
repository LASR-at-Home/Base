#!/usr/bin/env python3
from typing import List, Union, Optional
import time
import math
from collections import deque

import rclpy
import yasmin
import yasmin_ros
from yasmin import Blackboard, StateMachine
from yasmin_ros import set_ros_loggers, ServiceState

import message_filters
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from lasr_vision_interfaces.srv import YoloPoseDetection3D

# ==========================================
# 1. 3D 키포인트 추출 (Service Client State)
# ==========================================
class DetectKeypoints3D(ServiceState):
    def __init__(
        self,
        image_topic: str = "/head_front_camera/rgb/image_raw",
        depth_image_topic: str = "/head_front_camera/depth/image_raw",
        depth_camera_info_topic: str = "/head_front_camera/depth/camera_info",
        model: str = "yolo11n-pose.pt",
        confidence: float = 0.5,
        target_frame: str = "map",
        slop=0.1,
    ):
        super().__init__(
            srv_type=YoloPoseDetection3D,
            srv_name="/yolo/detect3d_pose",
            create_request_handler=self._create_req,
            outcomes=["succeeded", "failed"],
            response_handler=self.response_handler,
        )

        self.add_output_key("keypoint_detections_3d")
        self.add_output_key("image_raw")

        self.image_topic = image_topic
        self.depth_image_topic = depth_image_topic
        self.depth_camera_info_topic = depth_camera_info_topic
        self.model = model
        self.confidence = confidence
        self.target_frame = target_frame

        self.node = yasmin_ros.logger_node
        camera_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)

        self.data = None
        self.image_msg = None

        image_sub = message_filters.Subscriber(self.node, Image, self.image_topic, qos_profile=camera_qos)
        depth_sub = message_filters.Subscriber(self.node, Image, self.depth_image_topic, qos_profile=camera_qos)
        cam_info_sub = message_filters.Subscriber(self.node, CameraInfo, self.depth_camera_info_topic, qos_profile=camera_qos)

        self.cache = message_filters.Cache(cam_info_sub)
        self.ts = message_filters.ApproximateTimeSynchronizer([image_sub, depth_sub], queue_size=10, slop=slop)
        self.ts.registerCallback(self.callback)

    def callback(self, image_msg, depth_msg):
        self.data = (image_msg, depth_msg)

    def _create_req(self, blackboard):
        self.data = None
        self.image_msg = None
        deadline = time.time() + 5.0
        while self.data is None:
            if time.time() > deadline:
                self.node.get_logger().error("Timed out waiting for synced rgb/depth frames.")
                return "failed"
            time.sleep(0.1)

        image_msg, depth_msg = self.data
        self.image_msg = image_msg
        return YoloPoseDetection3D.Request(
            image_raw=image_msg,
            depth_image=depth_msg,
            depth_camera_info=self.cache.getLast(),
            model=self.model,
            confidence=self.confidence,
            target_frame=self.target_frame,
        )

    def response_handler(self, blackboard, response):
        if len(response.detections) == 0:
            return "failed"
        blackboard["keypoint_detections_3d"] = response
        return "succeeded"


# ==========================================
# 2. 자세 및 행동 판정 (Posture Evaluation State)
# ==========================================
class EvaluatePosture(yasmin.State):
    def __init__(self):
        super().__init__(outcomes=["succeeded", "failed"])
        # Waving(손 흔들기)를 인식하기 위해 최근 10프레임의 손목 좌표를 저장하는 버퍼
        self.r_wrist_y_history = deque(maxlen=10)
        self.l_wrist_y_history = deque(maxlen=10)

    def execute(self, blackboard):
        detections = blackboard.get("keypoint_detections_3d", None)
        if not detections or len(detections.detections) == 0:
            yasmin.YASMIN_LOG_WARN("No humans detected.")
            self.r_wrist_y_history.clear() # 사람이 없으면 버퍼 초기화
            self.l_wrist_y_history.clear()
            return "failed"

        # 첫 번째 사람 데이터 추출
        person = detections.detections[0]
        kp = {k.keypoint_name: k.point for k in person.keypoints}
        
        detected_posture = "Unknown"

        # 기본 관절이 모두 잡혔는지 확인 (에러 방지)
        if all(k in kp for k in ['right_wrist', 'right_shoulder', 'left_wrist', 'left_shoulder', 'right_hip', 'right_knee']):
            
            # --- 1. Dynamic Action: Waving (손 흔들기) ---
            # 손목이 어깨보다 높이 있을 때만 추적
            is_waving = False
            if kp['right_wrist'].z > kp['right_shoulder'].z:
                self.r_wrist_y_history.append(kp['right_wrist'].y)
                # 10프레임이 찼고, 좌우(Y축) 움직임 편차가 0.15m(15cm) 이상이면 Waving 판정
                if len(self.r_wrist_y_history) == 10:
                    movement = max(self.r_wrist_y_history) - min(self.r_wrist_y_history)
                    if movement > 0.15:
                        detected_posture = "Person Waving (Right Hand)"
                        is_waving = True
            
            if kp['left_wrist'].z > kp['left_shoulder'].z and not is_waving:
                self.l_wrist_y_history.append(kp['left_wrist'].y)
                if len(self.l_wrist_y_history) == 10:
                    movement = max(self.l_wrist_y_history) - min(self.l_wrist_y_history)
                    if movement > 0.15:
                        detected_posture = "Person Waving (Left Hand)"
                        is_waving = True

            # --- 2. Static Postures (정적 자세) ---
            if not is_waving:
                # 오른팔 올리기 / 왼팔 올리기
                if kp['right_wrist'].z > kp['right_shoulder'].z + 0.1:
                    detected_posture = "Person Raising Right Arm"
                elif kp['left_wrist'].z > kp['left_shoulder'].z + 0.1:
                    detected_posture = "Person Raising Left Arm"
                
                # 오른쪽 가리키기 (팔이 어깨 높이와 비슷하고, X/Y축으로 멀리 뻗었을 때)
                # 3D 공간에서 어깨와 손목의 거리가 0.4m 이상이고, 높이 차이가 적을 때
                elif math.hypot(kp['right_wrist'].x - kp['right_shoulder'].x, kp['right_wrist'].y - kp['right_shoulder'].y) > 0.4 \
                     and abs(kp['right_wrist'].z - kp['right_shoulder'].z) < 0.2:
                    detected_posture = "Person Pointing to the Right"
                
                # 서 있기 vs 앉아 있기 (골반과 무릎의 높이 차이)
                # 서 있으면 골반이 무릎보다 확연히 높음 (> 0.3m)
                elif (kp['right_hip'].z - kp['right_knee'].z) < 0.15:
                    detected_posture = "Person Sitting Down"
                else:
                    detected_posture = "Person Standing"

        # 결과 출력 및 블랙보드 저장
        yasmin.YASMIN_LOG_INFO(f"=== POSTURE DETECTED: {detected_posture} ===")
        blackboard["current_posture"] = detected_posture

        return "succeeded"


# ==========================================
# 3. State Machine 실행 (Main)
# ==========================================
def main():
    rclpy.init()
    set_ros_loggers()

    yasmin.YASMIN_LOG_INFO("Starting Human Posture Recognition SM...")
    sm = StateMachine(outcomes=["succeeded", "failed"], handle_sigint=True)
    bb = Blackboard()

    # 루프 구조: 탐지 -> 평가 -> 다시 탐지 (무한 반복)
    sm.add_state(
        "DETECT3D_POSE",
        DetectKeypoints3D(),
        transitions={"succeeded": "EVALUATE_POSTURE", "failed": "DETECT3D_POSE"}, # 실패해도 계속 시도
    )
    
    sm.add_state(
        "EVALUATE_POSTURE",
        EvaluatePosture(),
        transitions={"succeeded": "DETECT3D_POSE", "failed": "DETECT3D_POSE"}, # 끝나면 다시 탐지
    )
    
    outcome = sm(bb)
    yasmin.YASMIN_LOG_INFO(f"State Machine Finished with: {outcome}")

    if rclpy.ok():
        rclpy.shutdown()

if __name__ == "__main__":
    main()