#!/usr/bin/env python3
import smach
import smach_ros
import rospy
from sensor_msgs.msg import Image
#from lasr_skills.fake_nav import GoToLocation
from geometry_msgs.msg import Point
from lasr_skills import (
    GoToLocation,
    AskAndListen,
    DetectGesture,
    DetectClothing,
    DetectPose,
)
import navigation_helpers
import difflib
from geometry_msgs.msg import (
    Pose,
    PoseWithCovarianceStamped,
    Polygon,
    PoseStamped,
    Point,
    Quaternion,
)
from lasr_vision_msgs.msg import CDRequest, CDResponse
from lasr_vision_msgs.srv import (
    YoloDetection,
    YoloDetectionRequest
)
from cv_bridge import CvBridge
from image_geometry import PinholeCameraModel
from geometry_msgs.msg import PointStamped
import tf2_ros
import tf2_geometry_msgs

from typing import List, Literal
import itertools


class CountPeople(smach.StateMachine):

    class GetLocation(smach.State):
        def __init__(self):
            smach.State.__init__(
                self,
                outcomes=["succeeded", "failed"],
                input_keys=["location_index", "waypoints"],
                output_keys=["location"],
            )

        def execute(self, userdata) -> str:
            userdata.location = userdata.waypoints[userdata.location_index]
            return "succeeded"

    class ComputePath(smach.State):
        def __init__(self, waypoints: List[Pose]):
            smach.State.__init__(
                self,
                outcomes=["succeeded", "failed"],
                input_keys=["current_pose"],
                output_keys=["waypoints"],
            )
            self._waypoints = waypoints

        def execute(self, userdata) -> str:
            current_pose: Pose = rospy.wait_for_message(
                "/robot_pose", PoseWithCovarianceStamped
            ).pose.pose
            userdata.waypoints = navigation_helpers.min_hamiltonian_path(
                current_pose, self._waypoints
            )
            return "succeeded"


    class HandleDetections(smach.StateMachine):

        class GetResponse(smach.State):
            def __init__(self):
                smach.State.__init__(
                    self,
                    outcomes=["succeeded", "failed"],
                    #input_keys=["responses", "image_raw"],
                    input_keys=["responses", "image_raw", "depth_image", "camera_info"],
                    output_keys=["response", "responses", "person_point", "cropped_image"],
                )

            def execute(self, userdata):
                

                bridge = CvBridge()
                tf_buffer = tf2_ros.Buffer()
                tf_listener = tf2_ros.TransformListener(tf_buffer)

                detections = userdata.responses

                if not isinstance(detections, list) or len(detections) == 0:
                    rospy.logwarn("[YOLO] No 2D detections received.")
                    return "failed"

                detection = detections.pop(0)
                rospy.loginfo(f"[YOLO] Found detection: {detection.name} (conf: {detection.confidence:.2f})")

                # Optional: skip non-person detections
                if detection.name.lower() != "person":
                    rospy.loginfo(f"[YOLO] Skipping non-person detection: {detection.name}")
                    return "failed"

                try:
                    x, y, w, h = detection.xywh
                    u = int(x + w / 2)
                    v = int(y + h / 2)

                    # Convert depth image to OpenCV format
                    depth_image_cv = bridge.imgmsg_to_cv2(userdata.depth_image, desired_encoding="passthrough")

                    # Make sure the pixel is within bounds
                    if v >= depth_image_cv.shape[0] or u >= depth_image_cv.shape[1]:
                        raise ValueError("Pixel out of bounds")

                    depth = float(depth_image_cv[v, u]) / 1000.0  # mm → meters

                    if depth == 0.0 or depth != depth:  # NaN or missing
                        rospy.logwarn(f"[3D] Invalid depth at ({u}, {v}), using default 2D fallback.")
                        raise ValueError("invalid depth")

                    cam_model = PinholeCameraModel()
                    cam_model.fromCameraInfo(userdata.camera_info)
                    ray = cam_model.projectPixelTo3dRay((u, v))
                    point_cam = [r * depth for r in ray]

                    # Create a stamped point in camera frame
                    ps = PointStamped()
                    ps.header = userdata.camera_info.header
                    ps.point.x, ps.point.y, ps.point.z = point_cam

                    # Transform to 'map' frame
                    ps_world = tf_buffer.transform(ps, "map", rospy.Duration(1.0))
                    person_point = ps_world.point
                    rospy.loginfo(f"[3D] Person position: ({person_point.x:.2f}, {person_point.y:.2f}, {person_point.z:.2f})")


                except Exception as e:
                    rospy.logwarn(f"[3D] Depth projection failed: {e}")
                    # Fallback to 2D center point
                    if len(detection.xywh) == 4:
                        x, y, w, h = detection.xywh
                        person_point = Point(x=x + w / 2, y=y + h / 2, z=0.0)
                    else:
                        person_point = Point(0.0, 0.0, 0.0)

                # Assign outputs
                userdata.response = detection
                userdata.person_point = person_point
                userdata.cropped_image = userdata.image_raw  # no change here

                return "succeeded"



 



        class AddPerson(smach.State):
            def __init__(self):
                smach.State.__init__(
                    self,
                    outcomes=["succeeded"],
                    input_keys=["person_point", "all_people","detected_clothing"],
                    output_keys=["all_people"],
                )

            def execute(self, userdata):
                for existing_point, existing_clothing in userdata.all_people:
                    dist = navigation_helpers.euclidian_distance(existing_point, userdata.person_point)
                    rospy.loginfo(f"[ADD_PERSON] Distance to existing person: {dist:.2f}")
                    clothing_match = difflib.SequenceMatcher(
                        None,
                        existing_clothing.lower(),
                        userdata.detected_clothing.lower()
                    ).ratio()
                    rospy.loginfo(f"[ADD_PERSON] Clothing match score: {clothing_match:.2f}")

                    #if dist < 0.75 and clothing_match > 0.8:
                    if (userdata.person_point.z > 0 and dist < 0.75 and clothing_match > 0.8) or (userdata.person_point.z == 0 and clothing_match > 0.8):
                    
                        rospy.loginfo("[ADD_PERSON] Skipping duplicate person")
                        return "succeeded"

                userdata.all_people.append((userdata.person_point, userdata.detected_clothing))
                rospy.loginfo("[ADD_PERSON] Added new person")
                return "succeeded"


        

        def __init__(
            self,
            criteria: Literal["pose", "gesture", "clothes"],
            criteria_value: str,
        ):
            smach.StateMachine.__init__(
                self,
                outcomes=["succeeded", "failed"],
                #input_keys=["responses", "all_people"],
                #input_keys=["responses", "all_people", "image_raw"],
                input_keys=["responses", "all_people", "image_raw", "depth_image", "camera_info"],
                output_keys=["responses", "all_people"],
            )

            with self:

                if criteria == "gesture":
                    
                    smach.StateMachine.add(
                        "GET_RESPONSE",
                        self.GetResponse(),
                        transitions={
                            "succeeded": "DETECT_GESTURE",
                            "failed": "succeeded",
                        },
                    )
                    
                    smach.StateMachine.add(
                        "DETECT_GESTURE",
                        DetectGesture(criteria_value),
                        transitions={
                            "succeeded": "ADD_PERSON",
                            #"missing_keypoints": "GET_RESPONSE",
                            "failed": "GET_RESPONSE",
                        },
                        remapping={"img_msg": "cropped_image"},
                    )

                    smach.StateMachine.add(
                        "ADD_PERSON",
                        self.AddPerson(),
                        transitions={"succeeded": "GET_RESPONSE"},
                    )

                elif criteria == "pose":

                    smach.StateMachine.add(
                        "GET_RESPONSE",
                        self.GetResponse(),
                        transitions={
                            "succeeded": "DETECT_POSE",
                            "failed": "succeeded",
                        },
                    )

                    smach.StateMachine.add(
                        "DETECT_POSE",
                        DetectPose(criteria_value),
                        transitions={
                            "succeeded": "ADD_PERSON",
                            "failed": "GET_RESPONSE",
                        },
                        remapping={"img_msg": "cropped_image"},
                    )

                    smach.StateMachine.add(
                        "ADD_PERSON",
                        self.AddPerson(),
                        transitions={"succeeded": "GET_RESPONSE"},
                    )

                elif criteria == "clothes":

                    smach.StateMachine.add(
                        "GET_RESPONSE",
                        self.GetResponse(),
                        transitions={
                            "succeeded": "DETECT_CLOTHING",
                            "failed": "succeeded",
                        },
                    )

                    smach.StateMachine.add(
                        "DETECT_CLOTHING",
                        DetectClothing(criteria_value),
                        transitions={
                            "succeeded": "ADD_PERSON",
                            "failed": "GET_RESPONSE",
                        },
                        
                        remapping={"img_msg": "cropped_image", "detected_clothing": "detected_clothing"}

                    )
                    smach.StateMachine.add(
                        "ADD_PERSON",
                        self.AddPerson(),
                        transitions={"succeeded": "GET_RESPONSE"},
                    )

    class CountPeople(smach.State):

        def __init__(self, distance_threshold: float = 1.0):
            # Call the SMACH State constructor
            super().__init__(
                outcomes=["succeeded"],
                input_keys=["all_people"],
                output_keys=["people_count"],
            )
            # Now store your threshold
            self.distance_threshold = distance_threshold

        def execute(self, userdata):
            # Build a local list of unique people
            unique = []
            for point, clothing in userdata.all_people:
                if not any(
                    navigation_helpers.euclidian_distance(point, existing) < self.distance_threshold
                    for existing, _ in unique
                ):
                    unique.append((point, clothing))
    
            count = len(unique)
            userdata.people_count = count
            rospy.loginfo(f"[COUNT_PEOPLE] Final count: {count}")
            return "succeeded"


    def __init__(
        self,
        waypoints: List[Pose],
        polygon: Polygon,
        criteria: Literal["pose", "gesture", "clothes"],
        criteria_value: str,
    ):

        assert criteria in ["pose", "gesture", "clothes"], "Invalid criteria"

        if criteria == "gesture":
            assert criteria_value in [
                "raising_left_arm",
                "raising_right_arm",
                "pointing_to_the_right",
                "pointing_to_the_left",
                "waving",
            ], "Invalid gesture"
        elif criteria == "pose":
            assert criteria_value in [
                "sitting",
                "standing",
                "lying_down",
            ], "Invalid pose"
        elif criteria == "clothes":
            color_list = ["green", "blue", "yellow", "black", "white", "red", "orange", "gray", "brown"]
            clothe_list = ["t shirt", "shirt", "blouse", "sweater", "coat", "jacket"]
            clothes_list = [
                "t shirts",
                "shirts",
                "blouses",
                "sweaters",
                "coats",
                "jackets",
            ]
            color_clothe_list: List[str] = []
            for a, b in list(itertools.product(color_list, clothe_list)):
                color_clothe_list = color_clothe_list + [a + " " + b]
            color_clothes_list: List[str] = []
            for a, b in list(itertools.product(color_list, clothes_list)):
                color_clothes_list = color_clothes_list + [a + " " + b]
            assert (
                criteria_value in color_clothe_list + color_clothes_list
            ), "Invalid clothing"

        smach.StateMachine.__init__(
            self, outcomes=["succeeded", "failed"], output_keys=["people_count"]
        )

        with self:

            self.userdata.all_people = []

            smach.StateMachine.add(
                "COMPUTE_PATH",
                self.ComputePath(waypoints),
                transitions={"succeeded": "WAYPOINT_ITERATOR", "failed": "failed"},
            )

            waypoint_iterator = smach.Iterator(
                outcomes=["succeeded"],
                it=lambda: range(len(waypoints)),
                it_label="location_index",
                input_keys=["waypoints", "all_people"],
                output_keys=["all_people"],
                exhausted_outcome="succeeded",
            )

            with waypoint_iterator:
                container_sm = smach.StateMachine(
                    outcomes=["succeeded", "failed", "continue"],
                    input_keys=["waypoints", "location_index", "all_people"],
                    #output_keys=["all_people","image_raw", "responses"],
                    output_keys=["all_people","image_raw", "responses", "depth_image", "camera_info"],
                )

                with container_sm:
                    smach.StateMachine.add(
                        "GET_LOCATION",
                        self.GetLocation(),
                        transitions={
                            "succeeded": "GO_TO_LOCATION",
                            "failed": "continue",
                        },
                    )

                    smach.StateMachine.add(
                        "GO_TO_LOCATION",
                        GoToLocation(),
                        transitions={
                            "succeeded": "GET_IMAGE",
                            "failed": "failed",
                        },
                    )
                    
                    @smach.cb_interface(output_keys=["image_raw"], outcomes=["succeeded", "aborted"])
                    def get_image(userdata):
                        try:
                            userdata.image_raw = rospy.wait_for_message("/xtion/rgb/image_raw", Image, timeout=5.0)
                            return "succeeded"
                        except rospy.ROSException:
                            rospy.logerr("Timed out waiting for /xtion/rgb/image_raw")
                            return "aborted"

                    smach.StateMachine.add(
                        "GET_IMAGE",
                        smach.CBState(get_image),
                        transitions={
                            #"succeeded": "DETECT",
                            "succeeded": "GET_DEPTH",
                            "aborted": "continue"
                        },
                        remapping={"image_raw": "image_raw"},
                    )
                    @smach.cb_interface(output_keys=["depth_image"], outcomes=["succeeded", "aborted"])
                    def get_depth(userdata):
                        try:
                            userdata.depth_image = rospy.wait_for_message("/xtion/depth_registered/image_raw", Image, timeout=5.0)
                            return "succeeded"
                        except rospy.ROSException:
                            rospy.logerr("Timed out waiting for depth image")
                            return "aborted"

                    smach.StateMachine.add(
                        "GET_DEPTH",
                        smach.CBState(get_depth),
                        transitions={"succeeded": "GET_CAMERA_INFO", "aborted": "continue"},
                        remapping={"depth_image": "depth_image"},
                    )

                    @smach.cb_interface(output_keys=["camera_info"], outcomes=["succeeded", "aborted"])
                    def get_camera_info(userdata):
                        try:
                            from sensor_msgs.msg import CameraInfo
                            userdata.camera_info = rospy.wait_for_message("/xtion/rgb/camera_info", CameraInfo, timeout=5.0)
                            return "succeeded"
                        except rospy.ROSException:
                            rospy.logerr("Timed out waiting for camera info")
                            return "aborted"

                    smach.StateMachine.add(
                        "GET_CAMERA_INFO",
                        smach.CBState(get_camera_info),
                        transitions={"succeeded": "DETECT", "aborted": "continue"},
                        remapping={"camera_info": "camera_info"},
                    )

                    def make_yolo_request(userdata, request):
                        request.image_raw = userdata.image_raw
                        request.model = "yolo11n-seg.pt"
                        request.confidence = 0.5
                        request.filter = ["person"]
                        return request

                    smach.StateMachine.add(
                        "DETECT",
                        smach_ros.ServiceState(
                            "/yolo/detect",
                            YoloDetection,
                            request_cb=make_yolo_request,
                            input_keys=["image_raw"],
                            response_slots=["detected_objects"],
                        ),
                        transitions={
                            "succeeded": "HANDLE_DETECTIONS",
                            "aborted": "continue",
                            "preempted": "continue",
                        },
                        remapping={
                            "image_raw": "image_raw",
                            "detected_objects": "responses",
                        },
                    )

                    smach.StateMachine.add(
                        "HANDLE_DETECTIONS",
                        self.HandleDetections(criteria, criteria_value),
                        transitions={
                            "succeeded": "continue",
                            "failed": "continue",
                        },
                        remapping={
                            "image_raw": "image_raw",
                            "depth_image": "depth_image",
                            "camera_info": "camera_info",
                        },
                )
                waypoint_iterator.set_contained_state(
                    "CONTAINER_STATE", container_sm, 
                    loop_outcomes=["continue"]
                )
            smach.StateMachine.add(
                "WAYPOINT_ITERATOR", waypoint_iterator, {"succeeded": "COUNT_PEOPLE"}
            )

            smach.StateMachine.add(
                "COUNT_PEOPLE",
                self.CountPeople(),
                transitions={"succeeded": "succeeded"},
            )
