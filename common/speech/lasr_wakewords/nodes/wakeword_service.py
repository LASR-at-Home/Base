import rospy
import rospkg
import threading
import os
import time

import sounddevice as sd
import numpy as np
import openwakeword
from openwakeword.model import Model

from lasr_speech_recognition_msgs.srv import (
    WakewordTrigger,
    WakewordTriggerResponse,
    WakewordTriggerRequest,
)


class WakewordService:

    def __init__(self, device_index: int, sample_rate: int, frame_samples: int):
        self._device_index = device_index
        self._sample_rate = sample_rate
        self._frame_samples = frame_samples

        self._model_path = os.path.join(
            rospkg.RosPack().get_path("lasr_wakewords"), "models"
        )

        self._detect_wakeword_service = rospy.Service(
            "/lasr_wakewords/detect", WakewordTrigger, self._detect_wakeword
        )

        rospy.loginfo("/lasr_wakewords/detect is ready!")

    def _detect_wakeword(
        self, request: WakewordTriggerRequest
    ) -> WakewordTriggerResponse:
        wakeword = request.keyword.strip().lower()
        threshold = request.threshold
        max_duration = request.timeout
        detected = threading.Event()

        rospy.loginfo(
            f"Listening for wakeword '{wakeword}' with threshold {threshold} and timeout {max_duration}s"
        )

        model_path = os.path.join(self._model_path, f"{wakeword}.tflite")
        if not os.path.exists(model_path):
            rospy.logwarn(f"{model_path} does not exist.")
            return WakewordTriggerResponse(success=False)

        try:
            model = Model([model_path])
        except Exception as e:
            rospy.logerr(f"Failed to load model: {e}")
            return WakewordTriggerResponse(success=False)

        def audio_callback(indata, frames, time_info, status):
            if status:
                rospy.logwarn(f"Audio stream status: {status}")
            pcm = (indata[:, 0] * 32768).astype(np.int16)
            score = model.predict(pcm)[wakeword]
            rospy.loginfo(score)
            if score > threshold:
                rospy.loginfo(f"Wakeword '{wakeword}' detected (score={score:.3f})")
                detected.set()

        try:
            with sd.InputStream(
                device=self._device_index,
                channels=1,
                samplerate=self._sample_rate,
                blocksize=self._frame_samples,
                dtype="float32",
                callback=audio_callback,
            ):
                start_time = time.monotonic()

                while not rospy.is_shutdown() and not detected.is_set():
                    elapsed = time.monotonic() - start_time
                    if max_duration > 0 and elapsed >= max_duration:
                        rospy.loginfo(f"Timeout reached after {elapsed:.1f} seconds")
                        break
                    detected.wait(timeout=0.1)
        except Exception as e:
            rospy.logerr(f"Error opening InputStream: {e}")
            return WakewordTriggerResponse(success=False)

        return WakewordTriggerResponse(success=True)


if __name__ == "__main__":
    openwakeword.utils.download_models()
    rospy.init_node("lasr_wakewords_service")

    device_index = rospy.get_param("~device_index", 9)
    sample_rate = rospy.get_param("~sample_rate", 16000)
    frame_samples = rospy.get_param("~frame_samples", 1280)

    service = WakewordService(device_index, sample_rate, frame_samples)
    rospy.spin()
