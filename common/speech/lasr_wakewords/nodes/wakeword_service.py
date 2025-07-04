#!/usr/bin/env python3

import rospy
import sys

from lasr_speech_recoginition_msgs.srv import WakewordTrigger, WakewordTriggerResponse
import sounddevice as sd, numpy as np
from openwakeword.model import Model
import threading
import rospkg
import os
import openwakeword

# # Check available audio devices and their capabilities
# for i, dev in enumerate(sd.query_devices()):
#     try:
#         sd.check_input_settings(device=i, channels=1, samplerate=16000)
#         print(f"[{i}] {dev['name']} supports 16kHz mono input")
#     except Exception as e:
#         print(f"[{i}] {dev['name']} does NOT support 16kHz mono: {e}")

# Download model index (only needed once)
openwakeword.utils.download_models()

# Audio input configuration
DEVICE = 9  # Set this to the device index that supports 16kHz mono
SAMPLE_RATE = 16000
FRAME_SAMPLES = 1280

# Global state
model = None
WAKEWORD = None
detected = threading.Event()

# Locate package path
rospack = rospkg.RosPack()
pkg_path = rospack.get_path("lasr_wakewords")


# Audio callback for streaming audio to the model
def audio_callback(indata, frames, t, status):
    if status:
        rospy.logwarn(status)
    pcm = (indata[:, 0] * 32768).astype(np.int16)
    score = model.predict(pcm)[WAKEWORD]
    if score > 0.3:
        rospy.loginfo(f"Wake-word '{WAKEWORD}' detected (score={score:.3f})")
        detected.set()


# Service callback function
def handle_request(req):
    global model, WAKEWORD

    WAKEWORD = req.keyword.strip().lower()
    rospy.loginfo(f"Wakeword service called. Listening for '{WAKEWORD}'...")
    detected.clear()

    # Load corresponding model
    model_path = os.path.join(pkg_path, "model", f"{WAKEWORD}.tflite")
    if not os.path.exists(model_path):
        rospy.logerr(f"Model file not found: {model_path}")
        return WakewordTriggerResponse(success=False)

    model = Model([model_path])

    try:
        with sd.InputStream(
            device=DEVICE,
            channels=1,
            samplerate=SAMPLE_RATE,
            blocksize=FRAME_SAMPLES,
            dtype="float32",
            callback=audio_callback,
        ):
            while not rospy.is_shutdown() and not detected.wait(timeout=0.1):
                pass
    except Exception as e:
        rospy.logerr(f"Error opening InputStream: {e}")
        return WakewordTriggerResponse(success=False)

    return WakewordTriggerResponse(success=True)


if __name__ == "__main__":
    rospy.init_node("wakeword_listener_service")
    service = rospy.Service("wakeword_detect", WakewordTrigger, handle_request)
    rospy.loginfo("Wakeword listening service ready. Waiting for calls...")
    rospy.spin()
