#!/usr/bin python3
import os
import torch
import rclpy
from ament_index_python import packages

import sys
from pathlib import Path
import speech_recognition as sr
import numpy as np

import sounddevice  # needed to remove ALSA error messages
from lasr_speech_recognition_interfaces.srv import TranscribeAudio
from src import ModelCache # type: ignore

MODEL = "medium.en" # Whisper model
TIMEOUT = 5.0 # Timeout for listening for the start of a phrase
PHRASE_TIME_LIMIT = None # Timeout for listening for the end of a phrase

WHISPER_CACHE = os.path.join(str(Path.home()), '.cache', 'whisper')
os.makedirs(WHISPER_CACHE, exist_ok=True)
os.environ["TIKTOKEN_CACHE_DIR"] = WHISPER_CACHE

if len(sys.argv) < 3:
    print('Usage:')
    print('ros2 run lasr_speech_recognition transcribe_microphone by-index <device_index>')
    print('ros2 run lasr_speech_recognition transcribe_microphone by-name <substring>')
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

rclpy.init(args=sys.argv)
node = rclpy.create_node('transcribe_mic')

device = "cuda" if torch.cuda.is_available() else "cpu"
model_cache = ModelCache()
model = model_cache.load_model("medium.en", device=device)

# try to run inference on the example file
package_install = packages.get_package_prefix("lasr_speech_recognition_whisper")
package_root = os.path.abspath(os.path.join(package_install, os.pardir, os.pardir, "lasr_speech_recognition_whisper"))
example_fp = os.path.join(package_root, "test.m4a")
node.get_logger().info("Running transcription on example file to ensure model is loaded...")
transcription = model.transcribe(example_fp, fp16=torch.cuda.is_available())
node.get_logger().info(str(transcription))

microphone = sr.Microphone(device_index=device_index, sample_rate=16000)
r = sr.Recognizer()
with microphone as source:
    r.adjust_for_ambient_noise(source)

def handle_transcribe_audio(_):
    with microphone as source:

        wav_data = r.listen(source, timeout=TIMEOUT, phrase_time_limit=PHRASE_TIME_LIMIT).get_wav_data()
        float_data = np.frombuffer(wav_data, dtype=np.int16).astype(np.float32, order='C') / 32768.0

        phrase = model.transcribe(float_data, fp16=device == "cuda")["text"]
        return TranscribeAudio.Response(phrase=phrase)

node.create_service(TranscribeAudio, '/whisper/transcribe_audio', handle_transcribe_audio)

node.get_logger().info("Whisper service ready")
rclpy.spin(node)