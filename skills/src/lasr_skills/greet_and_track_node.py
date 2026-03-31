import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import os
import asyncio

from geometry_msgs.msg import Point
from sensor_msgs.msg import Image, CameraInfo
import message_filters

import smach
import smach_ros
from smach_ros import SimpleActionState, ServiceState, RosState

from lasr_vision_interfaces.action import EyeTracker
from lasr_vision_interfaces.srv import Recognise3D
from lasr_speech_recognition_interfaces.action import TranscribeSpeech
from lasr_llm_interfaces.srv import ReceptionistQueryLlm


class GetPersonLocationState(RosState):
    """SMACH State to get a person's 3D point from ReID vision and store it globally."""

    def __init__(self, node):
        super().__init__(
            node, outcomes=["succeeded", "failed"], output_keys=["person_point"]
        )
        self.node = node

        self.reid_client = self.node.create_client(
            Recognise3D, "/lasr_vision_reid/recognise/threed"
        )

        camera_name = "head_front_camera"
        self.image_sub = message_filters.Subscriber(
            self.node, Image, f"/{camera_name}/rgb/image_raw"
        )
        self.depth_sub = message_filters.Subscriber(
            self.node, Image, f"/{camera_name}/depth/image_raw"
        )
        self.camera_info_sub = message_filters.Subscriber(
            self.node, CameraInfo, f"/{camera_name}/depth/camera_info"
        )

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.depth_sub, self.camera_info_sub],
            queue_size=10,
            slop=2.0,
        )
        self.ts.registerCallback(self.vision_sync_callback)

        self.latest_vision_data = None
        self.got_first_vision_data = False

    def vision_sync_callback(
        self, image: Image, depth_image: Image, camera_info: CameraInfo
    ):
        self.latest_vision_data = (image, depth_image, camera_info)
        self.got_first_vision_data = True

    def execute(self, userdata):
        self.node.get_logger().info("Waiting for synchronized camera frames...")

        # Simple loop to wait for the first camera frame
        rate = self.node.create_rate(10)
        timeout = 0
        while not self.got_first_vision_data and timeout < 50:  # 5 second timeout
            rclpy.spin_once(self.node, timeout_sec=0.1)
            timeout += 1

        if not self.got_first_vision_data:
            self.node.get_logger().warn(
                "Camera frames not received in time. Taking default point."
            )
            userdata.person_point = Point(x=1.0, y=0.0, z=1.5)
            # return 'succeeded' anyway so the flow continues smoothly
            return "succeeded"

        self.node.get_logger().info("Camera frames acquired. Calling ReID service...")
        image, depth_image, camera_info = self.latest_vision_data

        req = Recognise3D.Request()
        req.image_raw = image
        req.depth_image = depth_image
        req.depth_camera_info = camera_info
        req.threshold = 0.5
        req.target_frame = "map"

        if not self.reid_client.wait_for_service(timeout_sec=3.0):
            self.node.get_logger().error("ReID service not available.")
            userdata.person_point = Point(x=1.0, y=0.0, z=1.5)
            return "failed"

        # Call service synchronously inside state execution
        future = self.reid_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)

        try:
            result = future.result()
            if result.detections:
                userdata.person_point = result.detections[0].point
                self.node.get_logger().info(
                    "Successfully grabbed 3D person point from ReID!"
                )
                return "succeeded"
            else:
                self.node.get_logger().warn(
                    "ReID succeeded but nobody was detected! Using default point."
                )
                userdata.person_point = Point(x=1.0, y=0.0, z=1.5)
                return "succeeded"
        except Exception as e:
            self.node.get_logger().error(f"ReID service call failed: {e}")
            userdata.person_point = Point(x=1.0, y=0.0, z=1.5)
            return "failed"


class SpeakGreetingState(RosState):
    """SMACH State to speak aloud using the system TTS."""

    def __init__(self, node):
        super().__init__(node, outcomes=["succeeded"])
        self.node = node

    def execute(self, userdata):
        self.node.get_logger().info("Speaking greeting prompt...")
        greeting_text = (
            "Hello guest! What is your name, and your favorite food and drink?"
        )
        os.system(f'espeak "{greeting_text}"')
        return "succeeded"


class ParseReceptionistLLMState(RosState):
    """SMACH State to parse text speech into structured data using LLM service."""

    def __init__(self, node):
        super().__init__(
            node, outcomes=["succeeded", "failed"], input_keys=["speech_sequence"]
        )
        self.node = node
        self.llm_client = self.node.create_client(
            ReceptionistQueryLlm, "receptionist_query_llm"
        )

    def execute(self, userdata):
        guest_phrase = userdata.speech_sequence
        if not guest_phrase:
            self.node.get_logger().warn("Empty speech received.")
            return "failed"

        self.node.get_logger().info(f"Asking LLM to parse: {guest_phrase}")

        req = ReceptionistQueryLlm.Request()
        req.llm_input = guest_phrase
        req.task = "name_and_interest"

        if not self.llm_client.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().error("LLM service not available.")
            return "failed"

        future = self.llm_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)

        try:
            result = future.result()
            self.node.get_logger().info(
                f"Extraction result - Name: {result.response.name}, Drink/Interests: {result.response.favourite_drink} {result.response.interests}"
            )
            return "succeeded"
        except Exception as e:
            self.node.get_logger().error(f"LLM service call failed: {e}")
            return "failed"


def build_greet_and_track_sm(node: Node) -> smach.StateMachine:
    """Builds the main state machine with concurrent branches."""
    sm = smach.StateMachine(outcomes=["succeeded", "failed"])

    with sm:
        # State 1: Retrieve face position
        smach.StateMachine.add(
            "GET_PERSON_LOCATION",
            GetPersonLocationState(node),
            transitions={"succeeded": "CONCURRENT_INTERACTION", "failed": "failed"},
        )

        def spawn_eye_tracking_goal(userdata, default_goal):
            goal = EyeTracker.Goal()
            goal.person_point = userdata.person_point
            return goal

        # Define Speech Goal
        speech_goal = TranscribeSpeech.Goal()
        speech_goal.energy_threshold = 300.0
        speech_goal.max_phrase_limit = 10.0

        def speech_result_cb(userdata, status, result):
            userdata.speech_sequence = result.sequence

        # State 2: Concurrent Tracking + Interacting
        sm_concurrence = smach.Concurrence(
            outcomes=["succeeded", "failed"],
            default_outcome="failed",
            input_keys=["person_point"],
            # For this scenario, we want the whole machine to end when the Interaction Sequence naturally finishes.
            # We don't care about the termination of the eye tracker (unless it actively fails)
            child_termination_cb=lambda outcome_map: (
                True if outcome_map["INTERACTION_SEQUENCE"] == "succeeded" else False
            ),
            outcome_cb=lambda outcome_map: (
                "succeeded"
                if outcome_map["INTERACTION_SEQUENCE"] == "succeeded"
                else "failed"
            ),
        )

        with sm_concurrence:
            # Thread A: Track Face
            smach.Concurrence.add(
                "EYE_TRACKING",
                SimpleActionState(
                    node,
                    "eye_tracker",
                    EyeTracker,
                    goal_cb=spawn_eye_tracking_goal,
                    input_keys=["person_point"],
                ),
            )

            # Thread B: Sequential Talk -> Listen -> Parse
            sm_interaction = smach.Sequence(
                outcomes=["succeeded", "failed"], connector_outcome="succeeded"
            )

            with sm_interaction:
                smach.Sequence.add("SPEAK_GREETING", SpeakGreetingState(node))
                smach.Sequence.add(
                    "LISTEN_TO_GUEST",
                    SimpleActionState(
                        node,
                        "transcribe_speech",
                        TranscribeSpeech,
                        goal=speech_goal,
                        result_cb=speech_result_cb,
                        output_keys=["speech_sequence"],
                    ),
                )
                smach.Sequence.add(
                    "PARSE_RESPONSE_LLM",
                    ParseReceptionistLLMState(node),
                    transitions={"failed": "failed"},
                )

            smach.Concurrence.add("INTERACTION_SEQUENCE", sm_interaction)

        smach.StateMachine.add(
            "CONCURRENT_INTERACTION",
            sm_concurrence,
            transitions={"succeeded": "succeeded", "failed": "failed"},
        )

    return sm


def main(args=None):
    rclpy.init(args=args)
    # The SMACH requires a node to attach its states and handles to.
    smach_node = rclpy.create_node("start_greet_and_track_sm")

    sm = build_greet_and_track_sm(smach_node)

    # Introduce the state machine introspector (useful for smach_viewer)
    sis = smach_ros.IntrospectionServer(
        "greet_and_track_sm_viewer", sm, "/START_GREET_AND_TRACK"
    )
    sis.start()

    try:
        smach_node.get_logger().info("Starting Greet And Track State Machine...")
        outcome = sm.execute()
        smach_node.get_logger().info(
            f"Greet And Track SM Finished with Outcome: {outcome}"
        )
    finally:
        sis.stop()
        smach_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
