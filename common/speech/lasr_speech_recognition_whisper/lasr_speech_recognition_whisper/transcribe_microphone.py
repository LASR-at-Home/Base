#!/usr/bin python3
import os
import sys
import torch
from pathlib import Path

import rclpy
from rclpy.node import Node
from ament_index_python import packages
from std_srvs.srv import Empty
from src import SpeechRecognitionToTopic, MicrophonePhraseCollector, ModelCache

WHISPER_CACHE = os.path.join(str(Path.home()), ".cache", "whisper")
os.makedirs(WHISPER_CACHE, exist_ok=True)
os.environ["TIKTOKEN_CACHE_DIR"] = WHISPER_CACHE


class TranscribeMicrophone(Node):
    def __init__(self):
        Node.__init__(self, "transcribe_microphone")
        self.worker = None
        self.collector = None

        self.create_service(Empty, "/whisper/adjust_for_noise", self.adjust_for_noise)
        self.create_service(Empty, "/whisper/start_listening", self.start_listening)
        self.create_service(Empty, "/whisper/stop_listening", self.stop_listening)

        self.get_logger().info("Starting the Whisper worker!")
        self.run_transcription()

    def run_transcription(self):
        if len(sys.argv) < 3:
            print("Usage:")
            print(
                "rosrun lasr_speech_recognition transcribe_microphone by-index <device_index>"
            )
            print(
                "rosrun lasr_speech_recognition transcribe_microphone by-name <substring>"
            )
            exit(1)
        else:
            matcher = sys.argv[1]
            device_index = None
            if matcher == "by-index":
                device_index = int(sys.argv[2])
            elif matcher == "by-name":
                import speech_recognition as sr

                microphones = enumerate(sr.Microphone.list_microphone_names())

                target_name = sys.argv[2]
                for index, name in microphones:
                    if target_name in name:
                        device_index = index
                        break

                if device_index is None:
                    print("Could not find device!")
                    exit(1)
            else:
                print("Invalid matcher")
                exit(1)

        self.collector = MicrophonePhraseCollector(device_index=device_index)
        self.collector.adjust_for_noise()

        model_cache = ModelCache()
        model = model_cache.load_model("medium.en")

        # try to run inference on the example file
        package_install = packages.get_package_prefix("lasr_speech_recognition_whisper")
        package_root = os.path.abspath(
            os.path.join(
                package_install, os.pardir, os.pardir, "lasr_speech_recognition_whisper"
            )
        )
        example_fp = os.path.join(package_root, "test.m4a")

        self.get_logger().info(
            "Running transcription on example file to ensure model is loaded..."
        )
        model_transcription = model.transcribe(
            example_fp, fp16=torch.cuda.is_available()
        )
        self.get_logger().info(str(model_transcription))

        self.worker = SpeechRecognitionToTopic(
            self.collector, model, "transcription", infer_partial=False
        )

    def adjust_for_noise(self, request, response):
        self.collector.adjust_for_noise()
        return response

    def start_listening(self, request, response):
        self.worker.start()
        return response

    def stop_listening(self, request, response):
        self.worker.stop()
        return response


def main(args=None):
    rclpy.init(args=args)
    transcribe_microphone = TranscribeMicrophone()
    rclpy.spin(transcribe_microphone)


if __name__ == "__main__":
    main()
