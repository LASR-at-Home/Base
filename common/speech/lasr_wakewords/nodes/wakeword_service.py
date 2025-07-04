from typing import Optional

import rospy
import rospkg

import threading
import os

from lasr_speech_recoginition_msgs.srv import (
    WakewordTrigger,
    WakewordTriggerResponse,
    WakewordTriggerRequest,
)

import sounddevice as sd
import numpy as np
import openwakeword
from openwakeword.model import Model


class WakewordService:

    # Parameters
    _device_index: int
    _sample_rate: int
    _frame_samples: int
    _wakeword_threshold: float

    # Detected event
    _detected: threading.Event

    # Current wakeword
    _wakeword: Optional[str]
    # Current model
    _model: Optional[Model]

    # Service
    _detect_wakeword_service: rospy.Service

    # Models
    _model_path: str = os.path.join(
        rospkg.RosPack().get_path("lasr_wakewords"), "models"
    )

    def __init__(
        self,
        device_index: int,
        sample_rate: int,
        frame_samples: int,
        wakeword_threshold: float,
    ):
        self._device_index = device_index
        self._sample_rate = sample_rate
        self._frame_samples = frame_samples
        self._wakeword_threshold = wakeword_threshold

        self._detected = threading.Event()
        self._wakeword = None
        self._model = None

        self._detect_wakeword_service = rospy.Service(
            "/lasr_wakewords/detect", WakewordTrigger, self._detect_wakeword
        )

        rospy.loginfo("/lasr_wakewords/detect is ready!")

    def _audio_callback(self, indata, frames, time, status):
        if status:
            rospy.logwarn(f"Audio stream status: {status}")
        pcm = (indata[:, 0] * 32768).astype(np.int16)
        score = self._model.predict(pcm)[self._wakeword]
        if score > self._wakeword_threshold:
            rospy.loginfo(f"Wakeword '{self._wakeword}' detected (score={score:.3f})")
            self._detected.set()

    def _detect_wakeword(
        self, request: WakewordTriggerRequest
    ) -> WakewordTriggerResponse:
        # Get wakeword
        self._wakeword = request.keyword.strip().lower()

        # Clear detected event
        self._detected.clear()

        rospy.loginfo(f"Listening for wakeword {self._wakeword}")

        model_path = os.path.join(self._model_path, f"{self._wakeword}.tflite")
        if not os.path.exists(model_path):
            rospy.logwarn(f"{model_path} does not exist.")
            return WakewordTriggerResponse(success=False)

        self._model = Model([model_path])

        try:
            with sd.InputStream(
                device=self._device_index,
                channels=1,
                samplerate=self._sample_rate,
                blocksize=self._frame_samples,
                dtype="float32",
                callback=self._audio_callback,
            ):
                while not rospy.is_shutdown() and not self._detected.wait(timeout=0.1):
                    pass
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
    wakeword_threshold = rospy.get_param("~wakeword_threshold", 0.3)
    wakeword_service = WakewordService(
        device_index, sample_rate, frame_samples, wakeword_threshold
    )
    rospy.spin()
