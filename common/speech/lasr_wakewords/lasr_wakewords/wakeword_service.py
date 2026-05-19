import os
import threading
import time
from typing import Dict, Sequence

from ament_index_python.packages import get_package_share_directory
import numpy as np
import openwakeword
from openwakeword.model import Model
import rclpy
from rclpy.node import Node
import sounddevice as sd

from lasr_speech_recognition_interfaces.srv import Wakeword


class WakewordService(Node):
    def __init__(self) -> None:
        super().__init__("lasr_wakewords_service")

        self.declare_parameter("device_index", 9)
        self.declare_parameter("sample_rate", 16000)
        self.declare_parameter("frame_samples", 1280)

        self._device_index = int(self.get_parameter("device_index").value)
        self._sample_rate = int(self.get_parameter("sample_rate").value)
        self._frame_samples = int(self.get_parameter("frame_samples").value)
        self._model_path = os.path.join(
            get_package_share_directory("lasr_wakewords"), "models"
        )

        self._detect_service = self.create_service(
            Wakeword, "/lasr_wakewords/detect", self._detect_wakeword
        )
        self.get_logger().info("/lasr_wakewords/detect is ready!")

    def _detect_wakeword(
        self, request: Wakeword.Request, response: Wakeword.Response
    ) -> Wakeword.Response:
        wakewords = list(request.keywords)
        threshold = float(request.threshold)
        max_duration = float(request.timeout)
        detected = threading.Event()
        detected_keyword = ""

        self.get_logger().info(
            "Listening for wakewords %s with threshold %.3f and timeout %.1fs"
            % (wakewords, threshold, max_duration)
        )

        model_paths = [
            os.path.join(self._model_path, f"{wakeword}.tflite")
            for wakeword in wakewords
        ]

        try:
            model = Model(model_paths)
        except Exception as exc:
            self.get_logger().error(f"Failed to load model: {exc}")
            response.success = False
            response.keyword = ""
            return response

        def audio_callback(indata, frames, time_info, status) -> None:
            nonlocal detected_keyword
            del frames, time_info
            if status:
                self.get_logger().warning(f"Audio stream status: {status}")
            pcm = (indata[:, 0] * 32768).astype(np.int16)
            result: Dict[str, float] = model.predict(pcm)
            self.get_logger().info(str(result))
            wakeword = max(result, key=lambda key: result[key])
            score = result[wakeword]
            if score > threshold:
                self.get_logger().info(
                    "Wakeword '%s' detected (score=%.3f)" % (wakeword, score)
                )
                detected_keyword = wakeword
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
                while rclpy.ok() and not detected.is_set():
                    elapsed = time.monotonic() - start_time
                    if max_duration > 0 and elapsed >= max_duration:
                        self.get_logger().info(
                            "Timeout reached after %.1f seconds" % elapsed
                        )
                        break
                    detected.wait(timeout=0.1)
        except Exception as exc:
            self.get_logger().error(f"Error opening InputStream: {exc}")
            response.success = False
            response.keyword = ""
            return response

        response.success = bool(detected_keyword)
        response.keyword = detected_keyword
        return response


def main(args: Sequence[str] | None = None) -> None:
    openwakeword.utils.download_models()
    rclpy.init(args=args)
    node = WakewordService()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
