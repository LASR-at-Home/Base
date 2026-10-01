#!/usr/bin/env python3
import os
import datetime
import threading
from pathlib import Path
from timeit import default_timer as timer
from typing import Optional

import numpy as np
import torch
import whisper
import soundfile as sf

import rclpy
from rclpy.node import Node
from rclpy.action.server import ActionServer, CancelResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor

from lasr_speech_recognition_interfaces.action import TranscribeSpeech
from lasr_speech_recognition_interfaces.srv import RecordAudio
from std_msgs.msg import String

SAMPLE_RATE = 16000


class TranscribeSpeechAction(Node):
    _result = TranscribeSpeech.Result()

    def __init__(self) -> None:
        super().__init__("transcribe_speech_action")

        self.declare_parameter("model", "small.en")
        self.declare_parameter("device", "cuda" if torch.cuda.is_available() else "cpu")
        self.declare_parameter("start_timeout", 5.0)
        self.declare_parameter("pause_threshold", 2.0)

        self.declare_parameter("condition_on_previous_text", False)
        self.declare_parameter("initial_prompt", "")
        self.declare_parameter("no_speech_threshold", 0.4)
        self.declare_parameter("logprob_threshold", -0.8)
        self.declare_parameter("temperature", [0.0, 0.2])
        self.declare_parameter("word_timestamps", True)
        self.declare_parameter("hallucination_silence_threshold", 0.5)

        self.declare_parameter("save_audio", True)
        self.declare_parameter("save_audio_dir", "/tmp/whisper_recordings")

        self._model_name = self.get_parameter("model").value
        self._device = self.get_parameter("device").value
        self._start_timeout = self.get_parameter("start_timeout").value
        self._pause_threshold = self.get_parameter("pause_threshold").value

        self._condition_on_previous_text = self.get_parameter(
            "condition_on_previous_text"
        ).value
        self._initial_prompt = self.get_parameter("initial_prompt").value
        self._no_speech_threshold = self.get_parameter("no_speech_threshold").value
        self._logprob_threshold = self.get_parameter("logprob_threshold").value
        self._temperature = self.get_parameter("temperature").value
        self._word_timestamps = self.get_parameter("word_timestamps").value
        self._hallucination_silence_threshold = self.get_parameter(
            "hallucination_silence_threshold"
        ).value

        self._save_audio = self.get_parameter("save_audio").value
        self._save_audio_dir = Path(self.get_parameter("save_audio_dir").value)
        if self._save_audio:
            self._save_audio_dir.mkdir(parents=True, exist_ok=True)
            self.get_logger().info(f"Saving audio to {self._save_audio_dir}")

        self._transcription_pub = self.create_publisher(
            String, "/live_speech_transcription", 10
        )

        self.get_logger().info(
            f"Loading Whisper model '{self._model_name}' on {self._device}..."
        )
        self._model = whisper.load_model(self._model_name, device=self._device)
        self.get_logger().info("Warming up Whisper...")
        self._model.transcribe(
            np.zeros(SAMPLE_RATE, dtype=np.float32), fp16=self._device == "cuda"
        )

        # The record call is waited on from inside execute_cb, so its response
        # must be handled by a different thread than the one running execute_cb.
        self._record_client = self.create_client(
            RecordAudio,
            "/microphone/record",
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.get_logger().info("Waiting for /microphone/record service...")
        self._record_client.wait_for_service()

        self._action_server = ActionServer(
            self,
            TranscribeSpeech,
            "transcribe_speech",
            execute_callback=self.execute_cb,
            cancel_callback=self.cancel_cb,
            callback_group=ReentrantCallbackGroup(),
        )

        self.get_logger().info(
            f"Whisper server ready (model={self._model_name}, device={self._device})"
        )

    def cancel_cb(self, goal_handle) -> CancelResponse:
        self.get_logger().info("Goal cancelled")
        return CancelResponse.ACCEPT

    def execute_cb(self, goal_handle):
        goal = goal_handle.request
        pause_threshold = (
            goal.max_phrase_limit
            if goal.max_phrase_limit > 0.0
            else self._pause_threshold
        )

        request = RecordAudio.Request(
            mode="phrase",
            start_timeout=float(self._start_timeout),
            pause_threshold=float(pause_threshold),
        )
        done = threading.Event()
        future = self._record_client.call_async(request)
        future.add_done_callback(lambda _: done.set())

        # The microphone node can't be interrupted mid-recording, so on cancel
        # we stop waiting and its response is discarded when it arrives.
        while not done.wait(timeout=0.1):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self._result.sequence = ""
                return self._result

        response = future.result()
        if response is None:
            self.get_logger().error(f"Record service error: {future.exception()}")
            self._result.sequence = ""
            goal_handle.abort()
            return self._result
        if not response.success:
            # Start timeout is not an error, it just means nobody spoke
            self.get_logger().warn(f"No audio recorded: {response.message}")
            self._result.sequence = ""
            goal_handle.succeed()
            return self._result

        try:
            float_data = np.frombuffer(response.samples, dtype=np.float32)
            start = timer()
            result = self._model.transcribe(
                float_data,
                fp16=(self._device == "cuda"),
                condition_on_previous_text=self._condition_on_previous_text,
                initial_prompt=self._initial_prompt,  # Context bias - Keyword that may be said
                no_speech_threshold=self._no_speech_threshold,  # Drops noise-only segments
                logprob_threshold=self._logprob_threshold,  # Filters low-confidence guesses
                temperature=tuple(
                    self._temperature
                ),  # Tuple of different thresholds to retry at
                word_timestamps=self._word_timestamps,
                hallucination_silence_threshold=self._hallucination_silence_threshold,  # Skips silent periods longer than this threshold
            )
            phrase = result.get("text", "").strip()
            self.get_logger().info(f"Transcribed in {timer() - start:.2f}s: '{phrase}'")
        except Exception as e:
            self.get_logger().error(f"Whisper error: {e}")
            self._result.sequence = ""
            goal_handle.abort()
            return self._result

        if phrase.lower() in {"", "you", "thank you.", "thanks.", "."}:
            self.get_logger().warn(f"Hallucination filtered: '{phrase}'")
            phrase = ""

        if self._save_audio and len(float_data) > 0:
            self._save_recording(float_data, phrase)

        self._transcription_pub.publish(String(data=phrase))
        self._result.sequence = phrase
        goal_handle.succeed()
        return self._result

    def _save_recording(self, float_data: np.ndarray, transcript: str) -> None:
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        wav_path = self._save_audio_dir / f"{stamp}.wav"
        txt_path = self._save_audio_dir / f"{stamp}.txt"
        try:
            sf.write(str(wav_path), float_data, SAMPLE_RATE, subtype="PCM_16")
            txt_path.write_text(transcript)
            self.get_logger().info(f"Saved recording: {wav_path.name}")
        except Exception as e:
            self.get_logger().warn(f"Failed to save recording: {e}")



def main(args=None):
    whisper_cache = os.path.join(str(Path.home()), ".cache", "whisper")
    os.makedirs(whisper_cache, exist_ok=True)
    os.environ["TIKTOKEN_CACHE_DIR"] = whisper_cache

    rclpy.init(args=args)
    server = TranscribeSpeechAction()
    executor = MultiThreadedExecutor()
    executor.add_node(server)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        server.destroy_node()
