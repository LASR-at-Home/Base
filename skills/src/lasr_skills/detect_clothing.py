#!/usr/bin/env python3
import smach
import rospy
from typing import Optional
from lasr_skills.clip_vqa import QueryImage

'''
class DecideClothing(smach.State):
    def __init__(self, desired_list: list):
        super().__init__(
            outcomes=["succeeded", "failed"],
            input_keys=["detected_clothing"]
        )
        self.desired_list = desired_list

    def execute(self, userdata) -> str:
        return "succeeded" if userdata.detected_clothing in self.desired_list else "failed"
        
'''
class DecideClothing(smach.State):
    def __init__(self, desired_list: list):
        super().__init__(
            outcomes=["succeeded", "failed"],
            input_keys=["detected_clothing"]
        )
        self.desired_list = desired_list

    def execute(self, userdata) -> str:
        detected = userdata.detected_clothing.strip().lower()
        rospy.loginfo(f"[CLIP-VQA] Detected clothing: '{detected}'")
        
        if detected in self.desired_list:
            return "succeeded"
        else:
            rospy.logwarn(f"[CLIP-VQA] Clothing '{detected}' not in desired list: {self.desired_list}")
            return "failed"

class DetectClothing(smach.StateMachine):
    def __init__(self, clothing_to_detect: Optional[str] = None):
        """
        Replace the old face-features pipeline with CLIP-VQA.
        clothing_to_detect should be e.g. "blue t shirt".
        """
        smach.StateMachine.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["img_msg"],
            output_keys=["detected_clothing"],
        )

        desired = clothing_to_detect

        with self:
            # 1) Ask CLIP-VQA “what clothing is this?”, restricting answers to your one desired string
            smach.StateMachine.add(
                "QUERY_CLOTHING",
                QueryImage(possible_answers=[desired]),
                transitions={
                    "succeeded": "DECIDE",
                    "aborted":   "failed",
                    "preempted": "failed",
                },
                remapping={
                    "img_msg": "img_msg",
                    "answer":  "detected_clothing",
                },
            )

            # 2) Check if the answer exactly matches your desired clothing string
            smach.StateMachine.add(
                "DECIDE",
                DecideClothing(desired),
                transitions={
                    "succeeded": "succeeded",
                    "failed":    "failed",
                },
                remapping={
                    "detected_clothing": "detected_clothing",
                },
            )
