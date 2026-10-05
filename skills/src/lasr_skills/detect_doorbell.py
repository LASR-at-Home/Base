import csv
import os
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
import yasmin
import yasmin_ros
from yasmin_ros.yasmin_node import YasminNode

from lasr_speech_recognition_interfaces.srv import RecordAudio


class DetectDoorbell(yasmin.State):
    TARGET_RATE = 16000  # What YAMNet needs
    REQUIRED_SAMPLES = 15600  # ~0.975s segments required by YAMNet
    RECORD_DURATION = 0.5  # seconds of audio requested per service call
    WAIT_FOR_DOORBELL_TIMEOUT = 15
    SERVICE_WAIT_TIMEOUT = 5.0

    def __init__(
        self, score_threshold=0.20,
        included_classes=[
                            "Buzzer",
                            "Telephone bell ringing",
                            "Alarm clock",
                            "Chink, clink",
                            "Smoke detector, smoke alarm",
                            "Beep, bleep",
                            "Bell",
                            "Bicycle bell",
                            "Glass",
                            "Percussion",
                            "Doorbell",
                            "Music",
                            "Glockenspiel",
                            "Marimba, xylophone",
                            "Chime",
                            "Fire alarm",
                            "Siren",
                            "Whistle",
                            "Air horn, truck horn",
                            "Vehicle horn, car horn, honking",
                            "Cowbell",
                            "Bagpipes"
                        ]
    ):
        super().__init__(outcomes=["succeeded", "failed"])
        self.score_threshold = score_threshold
        self.included_classes = included_classes

        # State variables
        self.model = None
        self.class_names = []
        self.audio_buffer = np.zeros(0, dtype=np.float32)

        self._node = YasminNode.get_instance()
        self._record_client = self._node.create_client(
            RecordAudio, "/microphone/record"
        )

        self.load_model()
        self.load_class_map()

    def record_audio(self):
        """Record a block from the microphone node and append it to the buffer.

        Returns False if the recording failed.
        """
        request = RecordAudio.Request(mode="fixed", duration=self.RECORD_DURATION)
        done = threading.Event()
        future = self._record_client.call_async(request)
        future.add_done_callback(lambda _: done.set())

        if not done.wait(timeout=self.RECORD_DURATION + self.SERVICE_WAIT_TIMEOUT):
            yasmin.YASMIN_LOG_WARN("Timed out waiting for /microphone/record")
            return False

        response = future.result()
        if response is None or not response.success:
            message = response.message if response else future.exception()
            yasmin.YASMIN_LOG_WARN(f"Recording failed: {message}")
            return False
        if response.sample_rate != self.TARGET_RATE:
            yasmin.YASMIN_LOG_ERROR(
                f"Microphone sample rate {response.sample_rate} != {self.TARGET_RATE}"
            )
            return False

        audio_chunk = np.frombuffer(response.samples, dtype=np.float32)
        # Only the latest window is ever used for inference
        self.audio_buffer = np.append(self.audio_buffer, audio_chunk)[
            -self.REQUIRED_SAMPLES :
        ]
        return True

    def load_model(self):
        # Imported here so that importing lasr_skills doesn't pull in TensorFlow
        import kagglehub
        import tensorflow as tf

        relative_path = Path("~/.cache/kagglehub/models/google/yamnet/tensorFlow2/yamnet/1")
        absolute_path = relative_path.expanduser().resolve()

        if os.path.exists(absolute_path):
            model_path = absolute_path
        else:
            model_path = kagglehub.model_download("google/yamnet/tensorFlow2/yamnet")

        self.model = tf.saved_model.load(model_path)

    def load_class_map(self):
        if self.model is None:
            raise RuntimeError("Model must be loaded before extracting the class map.")

        class_map_path = self.model.class_map_path().numpy().decode("utf-8")
        self.class_names = []
        with open(class_map_path, "r", encoding="utf-8") as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                self.class_names.append(row["display_name"])

    def run_inference(self):

        if len(self.audio_buffer) >= self.REQUIRED_SAMPLES:
            input_data = self.audio_buffer[-self.REQUIRED_SAMPLES :]

            scores, _, _ = self.model(input_data)

            mean_scores = np.mean(scores.numpy(), axis=0)
            top_class_index = np.argmax(mean_scores)
            top_score = mean_scores[top_class_index]
            prediction_name = self.class_names[top_class_index]

            if (
                top_score > self.score_threshold and prediction_name in self.included_classes

            ):
                yasmin_ros.logger_node.get_logger().info(
                    f" Detected: {prediction_name:<25} (Score: {top_score:.2f})"
                )
                return True

            return False

        return False

    def execute(self, blackboard):
        if not self._record_client.wait_for_service(
            timeout_sec=self.SERVICE_WAIT_TIMEOUT
        ):
            yasmin.YASMIN_LOG_ERROR("/microphone/record service is not available")
            return "failed"

        # Don't carry audio over from a previous run of this state
        self.audio_buffer = np.zeros(0, dtype=np.float32)
        t_end = time.time() + self.WAIT_FOR_DOORBELL_TIMEOUT

        while time.time() < t_end:
            if self.is_canceled():
                return "failed"
            if not self.record_audio():
                time.sleep(0.2)  # avoid spinning on a failing microphone
                continue
            if self.run_inference():
                return "succeeded"

        return "failed"


def main():
    rclpy.init()
    yasmin_ros.set_ros_loggers()

    # 1. Create the top-level state machine container
    sm = yasmin.StateMachine(outcomes=["succeeded", "failed"])

    # 2. Add your DetectDoorbell state instance to the state machine
    # The first state added to a YASMIN StateMachine automatically becomes the initial state
    sm.add_state(
        "DETECT_DOORBELL",
        DetectDoorbell(),
        transitions={
            "succeeded": "succeeded",  # Map state outcomes to machine outcomes
            "failed": "failed",
        },
    )

    try:
        # 3. Execute the state machine container
        outcome = sm()
        yasmin.YASMIN_LOG_INFO(f"State machine finished with outcome {outcome}")

    except Exception as e:
        yasmin.YASMIN_LOG_WARN(e)

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
