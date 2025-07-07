#!/usr/bin/env python3
import smach
import rospy
from typing import Optional
from lasr_skills.clip_vqa import QueryImage
from cv_bridge import CvBridge


class DecideClothing(smach.State):
    def __init__(self, desired_list: list):
        super().__init__(
            outcomes=["succeeded", "failed"],
            input_keys=["detected_clothing"]
        )
        self.desired_list = [s.lower() for s in desired_list]
        self.known_colors = [
            "red", "blue", "green", "yellow", "black", "white", "gray",
            "grey", "brown", "orange", "pink", "purple", "beige", "navy"
        ]

    def execute(self, userdata) -> str:
        detected = userdata.detected_clothing.strip().lower()
        rospy.loginfo(f"[CLIP-VQA] Detected clothing: '{detected}'")

        detected_color = next((c for c in self.known_colors if c in detected), None)
        if detected_color:
            rospy.loginfo(f"[CLIP-VQA] Extracted color: '{detected_color}'")
        else:
            rospy.logwarn(f"[CLIP-VQA] No known color found in detected clothing.")

        #if detected in self.desired_list:
        if any(desired in detected or detected in desired for desired in self.desired_list):
            return "succeeded"
        else:
            rospy.logwarn(f"[CLIP-VQA] '{detected}' does not match desired list: {self.desired_list}")
            return "failed"


class DetectClothing(smach.StateMachine):
    def __init__(self, clothing_to_detect: Optional[str] = None):
        smach.StateMachine.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["img_msg"],
            output_keys=["detected_clothing"],
        )

        bridge = CvBridge()

        # Dynamically generate possible answers
        #color, clothing_type = clothing_to_detect.split(" ", 1)

        cmd_clothes = clothing_to_detect.replace("_", " ")  # "blue t shirts"
        color, *clothing_words = cmd_clothes.split(" ")    # ["blue"], ["t","shirts"]
        clothing_type = " ".join(clothing_words)           # "t shirts"

        # 1) All colours you care about (add more as you like)
        ALL_COLORS = [
            "red", "blue", "green", "yellow", "black", "white",
            "gray", "brown", "orange", "pink", "purple", "beige"
        ]

        # 2) Clothing lists (singular & plural)
        singulars = ["t shirt", "shirt", "blouse", "sweater", "coat", "jacket"]
        plurals   = ["t shirts", "shirts", "blouses", "sweaters", "coats", "jackets"]


        color_variants = [color]
        

        '''
        clothing_variants = [clothing_type]
        if clothing_type == "t shirt":
            clothing_variants += ["shirt", "tee", "top"]

        possible_answers = [f"{c} {t}" for c in color_variants for t in clothing_variants]
        '''

        # 4) Build clothing variants by matching to your lists
        clothing_variants = []
        if clothing_type in plurals:
            # For "t shirts", also include singulars
            clothing_variants = [clothing_type]
            # find the matching singular (drop the trailing 's')
            singular_form = clothing_type.rstrip("s")
            if singular_form in singulars:
                clothing_variants.append(singular_form)
        elif clothing_type in singulars:
            clothing_variants = [clothing_type]
            # add plural
            plural_form = clothing_type + "s"
            if plural_form in plurals:
                clothing_variants.append(plural_form)
        else:
            # fallback: just use exactly what user said
            clothing_variants = [clothing_type]

        # 5) Combine into possible answers
        possible_answers = [f"{c} {t}" for c in color_variants for t in clothing_variants]

        # Define image conversion callback state
        @smach.cb_interface(input_keys=["img_msg"], output_keys=["img_msg"], outcomes=["succeeded", "failed"])
        def convert_to_bgr8(userdata):
            try:
                rospy.loginfo(f"[CONVERT_IMAGE] Original encoding: {userdata.img_msg.encoding}")
                cv_img = bridge.imgmsg_to_cv2(userdata.img_msg, desired_encoding="bgr8")
                converted_msg = bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
                userdata.img_msg = converted_msg
                return "succeeded"
            except Exception as e:
                rospy.logerr(f"[CONVERT_IMAGE] Failed: {e}")
                return "failed"

        # Add states inside the state machine
        with self:
            smach.StateMachine.add(
                "CONVERT_IMAGE",
                smach.CBState(convert_to_bgr8),
                transitions={"succeeded": "QUERY_CLOTHING", "failed": "failed"},
                remapping={"img_msg": "img_msg"},
            )

            smach.StateMachine.add(
                "QUERY_CLOTHING",
                QueryImage(possible_answers=possible_answers),
                transitions={
                    "succeeded": "DECIDE",
                    "aborted": "failed",
                    "preempted": "failed",
                },
                remapping={"img_msg": "img_msg", "answer": "detected_clothing"},
            )

            smach.StateMachine.add(
                "DECIDE",
                DecideClothing(clothing_to_detect),
                transitions={
                    "succeeded": "succeeded",
                    "failed": "failed",
                },
                remapping={"detected_clothing": "detected_clothing"},
            )
