import math
import rclpy
from rclpy.time import Time
import time
import traceback

import yasmin
from yasmin import State, Blackboard, StateMachine, Concurrence
import yasmin_ros
from yasmin_ros import ServiceState
from yasmin_viewer import YasminViewerPub

import tf2_ros

from std_msgs.msg import Header
from geometry_msgs.msg import (
    PointStamped,
    PoseWithCovarianceStamped,
    Pose,
    PolygonStamped,
    Point
)
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point
from shapely.geometry import Polygon as ShapelyPolygon

from lasr_skills import (
    DetectKeypoints3D,
    Wait,
    PlayMotion,
    Say,
    ContinuousGoToLocation,
    WaitForPersonInArea,
    LookToPoint,
    AskAndListen,
)

class CalculateDropPoint(ServiceState):
    def __init__(self):
        super().__init__(outcomes=["valid_point", "invalid_point", "failed"])
        self.add_input_key("keypoint_detections_3d")
        self.add_output_key("drop_point") 
         
        self.required_keypoints = ["right_elbow", "right_wrist"]

    def calcuate_point(self, elbow_point, wrist_point, angle_max=70.0):
        """ Returns (valid point, Point) """
        v_x = wrist_point.x - elbow_point.x
        v_y = wrist_point.y - elbow_point.y
        v_z = wrist_point.z - elbow_point.z

        t = -elbow_point.z / v_z

        drop_point = Point()
        drop_point.x = elbow_point.x + (t * v_x)
        drop_point.y = elbow_point.y + (t * v_y)
        drop_point.z = 0.0

        # Angle between z and pointing
        angle = math.degrees(
            math.asin(
                abs(v_z) / math.sqrt(v_x**2 + v_y**2 + v_z**2)
            )
        )
        yasmin.YASMIN_LOG_INFO(f"Pointing at {drop_point} at angle{angle} from z.")

        if v_z >= 0 or angle > angle_max:
            yasmin.YASMIN_LOG_INFO("Not Pointing at floor")
            return False, drop_point


        return True, drop_point
        

    def execute(self, blackboard):  
        try:
            # Use detect_keypoints to get points of right elbow and wrist. 
            # calcuate vector and follow vector untill it hits z=0 (floor) and retrive drop point
            
            # ----- Get elbow and wrist points
            elbow_point = None
            wrist_point = None
            for detection in blackboard["keypoint_detections_3d"].detections: # Assumed closest person is 
                kp = {k.keypoint_name: k.point for k in detection.keypoints}

                if "right_elbow" in kp and "right_wrist" in kp:
                    elbow_point = kp["right_elbow"]
                    wrist_point = kp["right_wrist"]
                    break
            
            if None in (elbow_point, wrist_point):
                return "invalid_point"

            # ----- Calculate drop point
            valid, drop_point = self.calcuate_point(elbow_point, wrist_point)

            if not valid:
                return "invalid_point"

            point_stamped = PointStamped()
            point_stamped.header.frame_id = "maP"
            point_stamped.header.stamp = Time().to_msg()
            point_stamped.point = drop_point
            blackboard["drop_point"] = point_stamped

            return "valid_point"
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"The following error occured: {e}")
            return "failed"
        
class PlaceBag(StateMachine):
    def __init__(self):
        # Outcomes align perfectly with your main locate_and_follow_host.py plan
        super().__init__(outcomes=["succedded", "failed"])

        self.add_state(
            "REQUEST_DROP_POINT",
            Say(text="With your right hand, please point where on the floor I should place the bag. "),
            transitions={
                "succeeded": "DETECT3D_POSE",
                "aborted": "DETECT3D_POSE",
                "canceled": "DETECT3D_POSE",
            },
        )
        self.add_state(
            "DETECT3D_POSE",
            DetectKeypoints3D(),
            transitions={
                "succeeded": "FIND_DROP_POINT", 
                "failed": "failed" # Some recovery then return back
            },
        )
        self.add_state(
            "FIND_DROP_POINT",
            CalculateDropPoint(),
            transitions={
                "valid_point": "LOOK_AT_DROP_POINT",
                "invalid_point": "REQUEST_NEW_POINT",
                "failed": "failed",
            },
        )

        self.add_state(
            "LOOK_AT_DROP_POINT",
            LookToPoint(),
            transitions={
                "succeeded": "ASK_TO_STEP_AWAY",
                "aborted": "failed",
                "canceled": "failed",
            },
            remappings={"pointstamped": "drop_point"}
        )

        self.add_state(
            "REQUEST_NEW_POINT",
            Say(text="I cannot place the bag there. Please point somewhere on the floor. "),
            transitions={
                "succeeded": "DETECT3D_POSE",
                "aborted": "DETECT3D_POSE",
                "canceled": "DETECT3D_POSE",
            },
        )

        self.add_state(
            "ASK_TO_STEP_AWAY",
            Say(text="Please step away. I will now place the bag."),
            transitions={
                "succeeded": "WAIT",
                "aborted": "WAIT",
                "canceled": "WAIT",
            },
        )

        self.add_state(
            "WAIT",
            Wait(5),  
            transitions={"succeeded": "READY_TO_PLACE", "failed": "failed"},
        )

        ## Navigate close to the point

        self.add_state(
            "READY_TO_PLACE",
            PlayMotion("unfold_arm"), # Motion goes heres
            transitions={
                "succeeded": "RELEASE_BAG",
                "aborted": "failed",
                "canceled": "failed",
            },
        )

        self.add_state(
            "RELEASE_BAG",
            PlayMotion("open"), # Motion goes heres
            transitions={
                "succeeded": "RESET",
                "aborted": "failed",
                "canceled": "failed",
            },
        )
        self.add_state(
            "RESET",
            PlayMotion("home"), # Motion goes heres
            transitions={
                "succeeded": "FINISH",
                "aborted": "failed",
                "canceled": "failed",
            },
        )

        self.add_state(
            "FINISH",
            Say(text="I have finished the task. "),
            transitions={
                "succeeded": "succeeded",
                "aborted": "failed",
                "canceled": "failed",
            },
        )

'''
    1. Ask person to point where on the floor to place the bag with right hand. 
    2. take keypoints of (elbow and wrist) pose and create vector to find point where to place
    3. Ask to step away
    4. Do playmotion of 'Drop Item'
    5. drive close enough to the point
    6. Open Gripper
    7. Home
    8. Go to end/ finish task. 

'''
## Later can adapt to check if they want to drop on table/ chair and if it is low enough ok if not request floor. 
