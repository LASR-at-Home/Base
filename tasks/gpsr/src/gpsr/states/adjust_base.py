#!/usr/bin/env python3
import rospy
import smach
from geometry_msgs.msg import Twist
from lasr_skills.vision import GetImage
from lasr_vision_msgs.srv import BodyPixDetection, BodyPixDetectionRequest
from lasr_vision_msgs.msg import BodyPixMaskRequest
from time import sleep


class Rotate(smach.State):
    def __init__(self, angular_speed, duration=1):
        smach.State.__init__(self, outcomes=["succeeded", "failed"])
        self.angular_speed = angular_speed
        self.duration = duration  # Duration for which to rotate

    def execute(self, userdata):
        rospy.loginfo("Rotating")
        pub = rospy.Publisher(
            "/mobile_base_controller/cmd_vel", Twist, queue_size=10, latch=True
        )

        # Construct the twist message with the desired angular speed
        twist = Twist()
        twist.angular.z = self.angular_speed

        try:
            # Rotate for the specified duration
            start_time = rospy.Time.now()
            while (rospy.Time.now() - start_time).to_sec() < self.duration:
                pub.publish(twist)
                sleep(0.1)  # Publish at 10 Hz

            # Stop the robot after rotating
            twist.angular.z = 0
            pub.publish(twist)
            return "succeeded"
        except rospy.ROSInterruptException:
            rospy.loginfo("ROS Interrupt Exception")
            return "failed"


class DetectHipCenter(smach.State):
    """
    State for detecting gestures.
    """

    def __init__(self, gesture_to_detect: str = "raising_left_hand"):
        self.gesture_to_detect = gesture_to_detect
        smach.State.__init__(
            self,
            outcomes=["move_left", "move_right", "stay_same", "failed"],
            input_keys=["img_msg"],
        )

        self.body_pix_client = rospy.ServiceProxy("/bodypix/detect", BodyPixDetection)

    def execute(self, userdata):

        body_pix_masks = BodyPixMaskRequest()
        body_pix_masks.parts = ["left_hip", "right_hip"]
        masks = [body_pix_masks]

        req = BodyPixDetectionRequest()
        req.image_raw = userdata.img_msg
        req.masks = masks
        req.dataset = "resnet50"
        req.confidence = 0.7

        try:
            res = self.body_pix_client(req)
        except Exception as e:
            print(e)
            return "failed"

        part_info = {}
        poses = res.poses

        if not poses:
            rospy.logerr("No poses available in the response.")
            return "failed"

        for pose in poses:
            for keypoint in pose.keypoints:
                if pose is None or pose.keypoints is None:
                    rospy.logerr("Pose or keypoints data is missing.")
                    return "failed"  # Skip this iteration if data is missing
                else:
                    part_info[keypoint.part] = {
                        "x": keypoint.xy[0],
                        "y": keypoint.xy[1],
                        "score": keypoint.score,
                    }

        print(part_info["leftHip"]["x"])
        print(part_info["rightHip"]["x"])
        if (part_info["leftHip"]["x"] + part_info["rightHip"]["x"]) / 2 < 300:
            return "move_left"
        elif (part_info["leftHip"]["x"] + part_info["rightHip"]["x"]) / 2 > 340:
            return "move_right"
        elif 300 < (part_info["leftHip"]["x"] + part_info["rightHip"]["x"]) / 2 < 340:
            return "stay_same"
        else:
            return "failed"


class AdjustBase(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed"])
        with self:
            smach.StateMachine.add(
                "GET_IMAGE",
                GetImage(),
                transitions={"succeeded": "BODY_PIX_DETECTION", "failed": "failed"},
            )

            smach.StateMachine.add(
                "BODY_PIX_DETECTION",
                DetectHipCenter(),
                transitions={
                    "move_left": "ADJUST_BASE_LEFT",
                    "move_right": "ADJUST_BASE_RIGHT",
                    "stay_same": "succeeded",
                    "failed": "GET_IMAGE",
                },
            )

            smach.StateMachine.add(
                "ADJUST_BASE_LEFT",
                Rotate(0.1),
                transitions={"succeeded": "GET_IMAGE", "failed": "failed"},
            )

            smach.StateMachine.add(
                "ADJUST_BASE_RIGHT",
                Rotate(-0.1),
                transitions={"succeeded": "GET_IMAGE", "failed": "failed"},
            )


if __name__ == "__main__":
    rospy.init_node("adjust_base")
    while not rospy.is_shutdown():
        sm = AdjustBase()
        sm.execute()
    rospy.spin()
