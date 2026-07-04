import os
import csv
import numpy as np
import pyaudio
import tensorflow as tf
import kagglehub
import yasmin
import yasmin_ros
import time
import rclpy

from scipy.signal import resample_poly  # Add this import at the top


class DetectDoorbell(yasmin.State):
    FORMAT = pyaudio.paInt16
    CHANNELS = 1
    TARGET_RATE = 16000  # What YAMNet needs
    HARDWARE_RATE = 44100  # Change this to match your hardware's supported rate (e.g., 44100 or 48000)
    CHUNK_SIZE = 1024
    REQUIRED_SAMPLES = 15600  # ~0.975s segments required by YAMNet
    WAIT_FOR_DOORBELL_TIMEOUT = 30

    def __init__(
        self, device_id=0, score_threshold=0.25, excluded_classes=["Speech", "Silence"]
    ):
        super().__init__(outcomes=["succeeded", "failed"])
        # Ensure correct hardware device ID discovered from the step 1 script
        self.device_id = device_id
        self.score_threshold = score_threshold
        self.excluded_classes = excluded_classes

        # State variables
        self.model = None
        self.class_names = []
        self.audio_interface = None
        self.stream = None
        self.audio_buffer = np.zeros(0, dtype=np.float32)

        self.load_model()
        self.load_class_map()
        self.initialize_audio()

    def initialize_audio(self):
        self.audio_interface = pyaudio.PyAudio()
        self.stream = self.audio_interface.open(
            format=self.FORMAT,
            channels=self.CHANNELS,
            rate=self.HARDWARE_RATE,  # Open at native hardware rate
            input=True,
            input_device_index=self.device_id,
            frames_per_buffer=self.CHUNK_SIZE,
        )

    def process_audio_frame(self):
        data = self.stream.read(self.CHUNK_SIZE, exception_on_overflow=False)
        # Convert raw buffer to float32
        audio_chunk = np.frombuffer(data, dtype=np.int16).astype(np.float32) / 32768.0

        # Resample from hardware rate (e.g., 44100) down to YAMNet target rate (16000)
        if self.HARDWARE_RATE != self.TARGET_RATE:
            audio_chunk = resample_poly(
                audio_chunk, self.TARGET_RATE, self.HARDWARE_RATE
            )

        yasmin_ros.logger_node.get_logger().info(
            f"Max amplitude: {np.max(np.abs(audio_chunk)):.4f}"
        )
        self.audio_buffer = np.append(self.audio_buffer, audio_chunk)

    def load_model(self):
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
        yasmin_ros.logger_node.get_logger().info(f"Run inference")

        if len(self.audio_buffer) >= self.REQUIRED_SAMPLES:
            input_data = self.audio_buffer[-self.REQUIRED_SAMPLES :]

            scores, _, _ = self.model(input_data)

            mean_scores = np.mean(scores.numpy(), axis=0)
            top_class_index = np.argmax(mean_scores)
            top_score = mean_scores[top_class_index]
            prediction_name = self.class_names[top_class_index]

            if (
                top_score > self.score_threshold
                and prediction_name not in self.excluded_classes
            ):
                yasmin_ros.logger_node.get_logger().info(
                    f" Detected: {prediction_name:<25} (Score: {top_score:.2f})"
                )
                return True

            self.audio_buffer = self.audio_buffer[-int(self.REQUIRED_SAMPLES / 2) :]
            return False

        return False

    def cleanup(self):
        print("\nStopping stream...")
        if self.stream:
            self.stream.stop_stream()
            self.stream.close()
        if self.audio_interface:
            self.audio_interface.terminate()
        print("Done.")

    def execute(self, blackboard):
        try:
            t_end = time.time() + self.WAIT_FOR_DOORBELL_TIMEOUT
            found = False
            yasmin_ros.logger_node.get_logger().info(f"Before loop")

            while time.time() < t_end and not found:
                yasmin_ros.logger_node.get_logger().info(f"Im loop")

                self.process_audio_frame()
                yasmin_ros.logger_node.get_logger().info(f"Processed audio frame loop")

                found = self.run_inference()
                yasmin_ros.logger_node.get_logger().info(f"found hqs vqlue {found}")
            if found:
                return "succeeded"
            else:
                return "failed"
        except KeyboardInterrupt:
            pass
        finally:
            self.cleanup()


def main():
    rclpy.init()
    yasmin_ros.set_ros_loggers()

    # 1. Create the top-level state machine container
    sm = yasmin.StateMachine(outcomes=["succeeded", "failed"])

    # 2. Add your DetectDoorbell state instance to the state machine
    # The first state added to a YASMIN StateMachine automatically becomes the initial state
    sm.add_state(
        "DETECT_DOORBELL",
        DetectDoorbell(device_id=0),
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
