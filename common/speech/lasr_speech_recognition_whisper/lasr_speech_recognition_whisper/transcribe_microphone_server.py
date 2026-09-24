#!/usr/bin/env python3
import os
import queue
import datetime
from collections import deque
from pathlib import Path
from timeit import default_timer as timer
from typing import Optional

import numpy as np
import torch
import whisper
import sounddevice as sd
import soundfile as sf

import rclpy
from rclpy.node import Node
from rclpy.action.server import ActionServer, CancelResponse
from rclpy.executors import ExternalShutdownException

from lasr_speech_recognition_interfaces.action import TranscribeSpeech
from std_msgs.msg import String

SAMPLE_RATE = 16000
CHUNK_SIZE = 512
MAX_PHRASE_CHUNKS = int(15.0 * SAMPLE_RATE / CHUNK_SIZE)
PRE_ROLL_CHUNKS = 16  # ~512 ms


class TranscribeSpeechAction(Node):
    _result = TranscribeSpeech.Result()

    def __init__(self) -> None:
        super().__init__("transcribe_speech_action")

        self.declare_parameter("model", "small.en")
        self.declare_parameter("device", "cuda" if torch.cuda.is_available() else "cpu")
        self.declare_parameter("mic_device", "default")
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
        self._mic_device = self.get_parameter("mic_device").value or None
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

        from silero_vad import load_silero_vad

        self._vad_model = load_silero_vad()

        self._audio_queue: queue.Queue = queue.Queue()
        self._pre_roll: deque = deque(maxlen=PRE_ROLL_CHUNKS)
        self._collecting = False

        self._stream = sd.InputStream(
            samplerate=SAMPLE_RATE,
            channels=1,
            dtype="float32",
            blocksize=CHUNK_SIZE,
            device=self._resolve_mic_device(),
            callback=self._audio_callback,
        )
        self._stream.start()

        self._action_server = ActionServer(
            self,
            TranscribeSpeech,
            "transcribe_speech",
            execute_callback=self.execute_cb,
            cancel_callback=self.cancel_cb,
        )

        self.get_logger().info(
            f"Whisper server ready (model={self._model_name}, device={self._device})"
        )

    def _resolve_mic_device(self) -> Optional[int]:
        if self._mic_device is None:
            return None
        if self._mic_device.isdigit():
            return int(self._mic_device)
        for idx, info in enumerate(sd.query_devices()):
            if self._mic_device in info["name"]:
                return idx
        raise ValueError(f"Could not find microphone: {self._mic_device}")

    def _audio_callback(
        self, indata: np.ndarray, frames: int, time_info, status
    ) -> None:
        chunk = indata[:, 0].copy()
        self._pre_roll.append(chunk)
        if self._collecting:
            self._audio_queue.put_nowait(chunk)

    def cancel_cb(self, goal_handle) -> CancelResponse:
        self.get_logger().info("Goal cancelled")
        self._collecting = False
        return CancelResponse.ACCEPT

    def execute_cb(self, goal_handle):
        goal = goal_handle.request
        pause_threshold = (
            goal.max_phrase_limit
            if goal.max_phrase_limit > 0.0
            else self._pause_threshold
        )
        max_silent_chunks = int(pause_threshold * SAMPLE_RATE / CHUNK_SIZE)
        max_start_chunks = int(self._start_timeout * SAMPLE_RATE / CHUNK_SIZE)

        self._vad_model.reset_states()
        self._audio_queue = queue.Queue()
        self._collecting = True

        speech_started = False
        silent_chunks = 0
        start_chunks_elapsed = 0
        collected_chunks = []

        try:
            while True:
                if goal_handle.is_cancel_requested:
                    self._collecting = False
                    goal_handle.canceled()
                    self._result.sequence = ""
                    return self._result

                try:
                    chunk = self._audio_queue.get(timeout=CHUNK_SIZE / SAMPLE_RATE)
                except queue.Empty:
                    continue

                is_speech = (
                    self._vad_model(
                        torch.from_numpy(chunk).unsqueeze(0), SAMPLE_RATE
                    ).item()
                    > 0.5
                )

                if not speech_started:
                    start_chunks_elapsed += 1
                    if start_chunks_elapsed > max_start_chunks:
                        self.get_logger().warn("Start timeout — no speech detected.")
                        self._collecting = False
                        self._result.sequence = ""
                        goal_handle.succeed()
                        return self._result
                    if is_speech:
                        speech_started = True
                        collected_chunks = list(self._pre_roll) + [chunk]
                else:
                    collected_chunks.append(chunk)
                    if is_speech:
                        silent_chunks = 0
                    else:
                        silent_chunks += 1
                        if silent_chunks >= max_silent_chunks:
                            break
                    if len(collected_chunks) >= MAX_PHRASE_CHUNKS:
                        self.get_logger().warn("Max phrase duration reached.")
                        break

        except Exception as e:
            self.get_logger().error(f"Audio collection error: {e}")
            self._collecting = False
            self._result.sequence = ""
            goal_handle.abort()
            return self._result
        finally:
            self._collecting = False

        try:
            float_data = np.concatenate(collected_chunks)
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

    def destroy_node(self):
        self._stream.stop()
        self._stream.close()
        super().destroy_node()

    def destroy_node(self):
        self._stream.stop()
        self._stream.close()
        super().destroy_node()


def main(args=None):
    whisper_cache = os.path.join(str(Path.home()), ".cache", "whisper")
    os.makedirs(whisper_cache, exist_ok=True)
    os.environ["TIKTOKEN_CACHE_DIR"] = whisper_cache

    rclpy.init(args=args)
    server = TranscribeSpeechAction()
    try:
        rclpy.spin(server)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        server.destroy_node()
