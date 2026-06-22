import math
import rclpy
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy


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
from visualization_msgs.msg import Marker
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point
from shapely.geometry import Polygon as ShapelyPolygon

from lasr_skills import (
    DetectKeypoints3D,
    Wait,
    PlayMotion,
    Say,
    Rotate,
    FollowPerson,
    LookToPoint,
    AskAndListen,
)
import random

class CalculateDropPoint(State):
    def __init__(self):
        super().__init__(outcomes=["valid_point", "invalid_point", "failed"])
        self.add_input_key("keypoint_detections_3d")
        self.add_output_key("drop_point") 
        
        self.node = yasmin_ros.logger_node 
        self.debug_pub = self.node.create_publisher(
            Marker,
            "/place_bag/debug/drop_point",
            10,
        ) 

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
        

        # DEBUG
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.node.get_clock().now().to_msg()
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position = drop_point

        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.debug_pub.publish(marker)

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
            point_stamped.header.frame_id = "map"
            point_stamped.header.stamp = Time().to_msg()
            point_stamped.point = drop_point
            blackboard["drop_point"] = point_stamped

            return "valid_point"
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"The following error occured: {e}")
            return "failed"


class PlacingMotion(StateMachine):
    def __init__(self, outcomes=["succeeded", "failed"]):
        self.add_input_key("drop_point")

        self.add_state(
            "PRE_NAV",
            PlayMotion("pre_navigation"),
            transitions={
                "succeeded": "FACE_DROP_POINT",
                "aborted": "failed",
                "canceled": "failed",
            },
        )

        self.add_state(
            "FACE_DROP_POINT",
            Rotate(mode="point"),
            transitions={"succeeded": "LOOK_AT_DROP_POINT", "failed": "failed"},
            remappings={"target_point": "drop_point"}
        )

        self.add_state(
            "LOOK_AT_DROP_POINT",
            LookToPoint(),
            transitions={
                "succeeded": "PLACE_MOTION",
                "aborted": "failed",
                "canceled": "failed",
                "timeout": "failed",
            },
            remappings={"pointstamped": "drop_point"}
        ) 


        self.add_state(
            "PLACE_MOTION",
            PlayMotion("reach_arm_vertical_gripper"), # Motion goes heres
            transitions={
                "succeeded": "PLAYMOTION_BREAK_1",
                "aborted": "failed",
                "canceled": "failed",
            },
        )
        self.add_state(
            "PLAYMOTION_BREAK_1",
            Wait(1),  
            transitions={"succeeded": "RELEASE_BAG", "failed": "failed"},
        )

        self.add_state(
            "RELEASE_BAG",
            PlayMotion("open"), # Motion goes heres
            transitions={
                "succeeded": "PLAYMOTION_BREAK_2",
                "aborted": "failed",
                "canceled": "failed",
            },
        )
        self.add_state(
            "PLAYMOTION_BREAK_2",
            Wait(1),  
            transitions={"succeeded": "RESET", "failed": "failed"},
        )
        self.add_state(
            "RESET",
            PlayMotion("home"), # Motion goes heres
            transitions={
                "succeeded": "CLOSE_GRIPPER",
                "aborted": "failed",
                "canceled": "failed",
            },
        )
        self.add_state(
            "CLOSE_GRIPPER",
            PlayMotion("close"), # Motion goes heres
            transitions={
                "succeeded": "LOOK_CENTER",
                "aborted": "failed",
                "canceled": "failed",
            },
        )
        self.add_state(
            "LOOK_CENTER",
            PlayMotion("look_center"), # Motion goes heres
            transitions={
                "succeeded": "succeeded",
                "aborted": "failed",
                "canceled": "failed",
            },
        )

class PlaceBag(StateMachine):
    def __init__(self):
        # Outcomes align perfectly with your main locate_and_follow_host.py plan
        super().__init__(outcomes=["succeeded", "failed"])

        self.add_state(
            "POST_NAV",
            PlayMotion("post_navigation"),
            transitions={
                "succeeded": "REQUEST_DROP_POINT",
                "aborted": "failed",
                "canceled": "failed",
            },
        )

        self.add_state(
            "REQUEST_DROP_POINT",
            Say(text="With your right hand, please point where on the floor I should place the bag. "),
            transitions={
                "succeeded": "WAIT_FOR_POINT",
                "aborted": "WAIT_FOR_POINT",
                "canceled": "WAIT_FOR_POINT",
            },
        )
        self.add_state(
            "WAIT_FOR_POINT",
            Wait(3),  
            transitions={"succeeded": "DETECT3D_POSE", "failed": "failed"},
        )
        self.add_state(
            "DETECT3D_POSE",
            DetectKeypoints3D(),
            transitions={
                "succeeded": "FIND_DROP_POINT", 
                "failed": "NO_POSE_FOUND" # Some recovery then return back
            },
        )
        self.add_state(
            "NO_POSE_FOUND",
            Say(text="I can't see where you are pointing properly. I will try again. "),
            transitions={
                "succeeded": "WAIT_FOR_POINT",
                "aborted": "WAIT_FOR_POINT",
                "canceled": "WAIT_FOR_POINT",
            },
        )

        self.add_state(
            "FIND_DROP_POINT",
            CalculateDropPoint(),
            transitions={
                "valid_point": "LOOK_AT_DROP_POINT_1",
                "invalid_point": "REQUEST_NEW_POINT",
                "failed": "failed",
            },
        )

        self.add_state(
            "LOOK_AT_DROP_POINT_1",
            LookToPoint(),
            transitions={
                "succeeded": "ASK_TO_STEP_AWAY",
                "aborted": "failed",
                "canceled": "failed",
                "timeout": "failed",
            },
            remappings={"pointstamped": "drop_point"}
        ) 

        #TODO:  Add a 3d detect in area check to ensure area is empty

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
            Say(text="Please step away. I will wait a few seconds, then place the bag."),
            transitions={
                "succeeded": "WAIT",
                "aborted": "WAIT",
                "canceled": "WAIT",
            },
        )

        self.add_state(
            "WAIT",
            Wait(3),  
            transitions={"succeeded": "PLACE_BAG_MOTION", "failed": "failed"},
        )

        # Add a sweep here and make a seperate SM for this

        #TODO:  Navigate close to the point

        self.add_state(
            "PLACE_BAG_MOTION",
            PlacingMotion(),
            transitions={"succeeded": "FINISH", "failed": "failed"}
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

def main():
    rclpy.init()

    yasmin_ros.set_ros_loggers()

    # sm = StateMachine(outcomes=["succeeded", "failed"])
    # sm.add_state(
    #     "CALL_HOST",
    #     Say(text="I have a bag. Can the host stand infront of me to lead the way."),
    #     transitions={
    #         "succeeded": "FOLLOW_HOST",
    #         "aborted": "failed",
    #         "canceled": "failed",
    #     },
    # )

    # sm.add_state(
    #     "FOLLOW_HOST",
    #     FollowPerson(),
    #     transitions={
    #         "succeeded": "PLACE_BAG",
    #         "failed": "failed",
    #     },
    # )

    # sm.add_state(
    #     "PLACE_BAG",
    #     PlaceBag(),
    #     transitions={
    #         "succeeded": "succeeded",
    #         "failed": "failed",
    #     },
    # )
    sm = PlaceBag()
    sm.set_sigint_handler(True)
    bb = Blackboard()
    bb["z_sweep_min"] = -10
    bb["z_sweep_max"] = 50

    YasminViewerPub(sm, "Follow_Person")

    outcome = sm(bb)

    yasmin.YASMIN_LOG_INFO(outcome)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
