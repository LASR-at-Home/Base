#!/usr/bin/env python3

import rospy
import sys
print(f"Interpreter: {sys.executable}")
print(f"Script path: {__file__}")

from lasr_wakewords.srv import WakewordTrigger, WakewordTriggerResponse
import sounddevice as sd, numpy as np
from openwakeword.model import Model
import threading
import rospkg
import os

# print(sd.query_devices()) # find the correct device index

DEVICE = 4
SAMPLE_RATE = 16000
FRAME_SAMPLES = 1280
WAKEWORD = "no"

rospack = rospkg.RosPack()
pkg_path = rospack.get_path('lasr_wakewords')
model_path = os.path.join(pkg_path, 'models', f"{WAKEWORD}.tflite")
model = Model([model_path])

# Global detection state
detected = threading.Event()

def audio_callback(indata, frames, t, status):
    if status:
        rospy.logwarn(status)
    pcm = (indata[:, 0] * 32768).astype(np.int16)
    score = model.predict(pcm)[WAKEWORD]
    if score > 0.3:
        rospy.loginfo(f"Wake-word '{WAKEWORD}' detected (score={score:.3f})")
        detected.set()

def handle_request(req):
    rospy.loginfo("Wakeword service called. Listening...")
    detected.clear()

    with sd.InputStream(device=DEVICE, channels=1,
                        samplerate=SAMPLE_RATE, blocksize=FRAME_SAMPLES,
                        dtype="float32", callback=audio_callback):
        while not rospy.is_shutdown() and not detected.wait(timeout=0.1):
            pass

    return WakewordTriggerResponse(success=True)

if __name__ == "__main__":
    rospy.init_node("wakeword_listener_service")
    service = rospy.Service("wakeword_detect", WakewordTrigger, handle_request)
    rospy.loginfo("Wakeword listening service ready. Waiting for calls...")
    rospy.spin()
