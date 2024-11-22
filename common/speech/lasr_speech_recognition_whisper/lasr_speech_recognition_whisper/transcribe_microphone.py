#!/usr/bin python3
import os
import torch
from ament_index_python import packages
from pathlib import Path

WHISPER_CACHE = os.path.join(str(Path.home()), '.cache', 'whisper')
os.makedirs(WHISPER_CACHE, exist_ok=True)
os.environ["TIKTOKEN_CACHE_DIR"] = WHISPER_CACHE

import sys

# TODO port node

if len(sys.argv) < 3:
    print('Usage:')
    print('rosrun lasr_speech_recognition transcribe_microphone by-index <device_index>')
    print('rosrun lasr_speech_recognition transcribe_microphone by-name <substring>')
    exit(1)
else:
    matcher = sys.argv[1]
    device_index = None
    if matcher == 'by-index':
        device_index = int(sys.argv[2])
    elif matcher == 'by-name':
        import speech_recognition as sr
        microphones = enumerate(sr.Microphone.list_microphone_names())

        target_name = sys.argv[2]
        for index, name in microphones:
            if target_name in name:
                device_index = index
                break
        
        if device_index is None:
            print('Could not find device!')
            exit(1)
    else:
        print('Invalid matcher')
        exit(1)

import rclpy
from std_srvs.srv import Empty, EmptyResponse

with rclpy.init(args=None):
    node = rclpy.create_node('transcribe_mic')  # was anonymous in ROS1

from lasr_speech_recognition_whisper import SpeechRecognitionToTopic, MicrophonePhraseCollector, load_model

collector = MicrophonePhraseCollector(device_index=device_index)
collector.adjust_for_noise()

#model = load_model("base.en")

model = load_model("medium.en")

# try to run inference on the example file
package_install = packages.get_package_prefix("lasr_speech_recognition_whisper")
package_root = os.path.abspath(os.path.join(package_install, os.pardir, os.pardir, "lasr_speech_recognition_whisper"))
example_fp = os.path.join(package_root, "test.m4a")

node.get_logger().info("Running transcription on example file to ensure model is loaded...")
node.get_logger().info(model.transcribe(example_fp, fp16=torch.cuda.is_available()))

worker = SpeechRecognitionToTopic(collector, model, "transcription", infer_partial = False)

def adjust_for_noise(_):
    collector.adjust_for_noise()
    return EmptyResponse()

def start_listening(_):
    worker.start()
    return EmptyResponse()

def stop_listening(_):
    worker.stop()
    return EmptyResponse()

node.create_service(Empty, '/whisper/adjust_for_noise', adjust_for_noise)
node.create_service(Empty, '/whisper/start_listening', start_listening)
node.create_service(Empty, '/whisper/stop_listening', stop_listening)

node.get_logger().info("Starting the Whisper worker!")
rclpy.spin(node)
