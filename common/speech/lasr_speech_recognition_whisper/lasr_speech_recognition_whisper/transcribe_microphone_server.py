#!/usr/bin python3
import os
import sounddevice  # needed to remove ALSA error messages
import argparse
from typing import Optional
from dataclasses import dataclass
from pathlib import Path
from timeit import default_timer as timer

import numpy as np
import torch

import rclpy
from rclpy.node import Node
from rclpy.action.server import ActionServer, CancelResponse

import speech_recognition as sr  # type: ignore
from lasr_speech_recognition_interfaces.action import TranscribeSpeech  # type: ignore
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String  # type: ignore
from src import ModelCache  # type: ignore

# TODO: argpars -> ROS2 params, test behaviour of preemption


@dataclass
class speech_model_params:
    """Class for storing speech recognition model parameters.

    Args:
        model_name (str, optional): Name of the speech recognition model. Defaults to "medium.en".
        Must be a valid Whisper model name.
        device (str, optional): Device to run the model on. Defaults to "cuda" if available, otherwise "cpu".
        start_timeout (float): Max number of seconds of silence when starting listening before stopping. Defaults to 5.0.
        phrase_duration (Optional[float]): Max number of seconds of the phrase. Defaults to 10 seconds.
        sample_rate (int): Sample rate of the microphone. Defaults to 16000Hz.
        mic_device (Optional[str]): Microphone device index or name. Defaults to None.
        timer_duration (Optional[int]): Duration of the timer for adjusting the microphone for ambient noise. Defaults to 20 seconds.
        warmup (bool): Whether to warmup the model by running inference on a test file. Defaults to True.
        energy_threshold (Optional[int]): Energy threshold for silence detection. Using this disables automatic adjustment. Defaults to None.
        pause_threshold (Optional[float]): Seconds of non-speaking audio before a phrase is considered complete. Defaults to 0.8 seconds.
    """

    model_name: str = "medium.en"
    device: str = "cuda" if torch.cuda.is_available() else "cpu"
    start_timeout: float = 5.0
    phrase_duration: Optional[float] = 10
    sample_rate: int = 16000
    mic_device: Optional[str] = None
    timer_duration: Optional[int] = 20
    warmup: bool = True
    energy_threshold: Optional[int] = None
    pause_threshold: Optional[float] = 2.0


class TranscribeSpeechAction(Node):
    # create messages that are used to publish feedback/result
    _feedback = TranscribeSpeech.Feedback()
    _result = TranscribeSpeech.Result()

    def __init__(
        self,
        action_name: str,
        model_params: speech_model_params,
    ) -> None:
        """Starts an action server for transcribing speech.

        Args:
            action_name (str): Name of the action server.
        """
        Node.__init__(self, "transcribe_speech_action")
        self._action_name = action_name
        self._model_params = model_params
        self._transcription_server = self.create_publisher(
            String, "/live_speech_transcription", 10
        )

        model_cache = ModelCache()
        self._model = model_cache.load_model(
            self._model_params.model_name,
            self._model_params.device,
            self._model_params.warmup,
        )
        # Configure the speech recogniser object and adjust for ambient noise
        self.recogniser = self._configure_recogniser()

        # Set up the action server and register execution callback
        self._action_server = ActionServer(
            self,
            TranscribeSpeech,
            self._action_name,
            execute_callback=self.execute_cb,
            cancel_callback=self.cancel_cb,
            # auto_start=False,  # not required in ROS2 ?? (cb is async)
        )
        self._action_server.register_cancel_callback(self.cancel_cb)
        self._listening = False

        # self._action_server.start()  # not required in ROS2
        self.get_logger().info(f"Speech Action server {self._action_name} started")

    def _configure_microphone(self) -> sr.Microphone:
        """Configures the microphone for listening to speech based on the
        microphone device index or name.

        Returns: microphone object
        """

        if self._model_params.mic_device is None:
            # If no microphone device is specified, use the system default microphone
            return sr.Microphone(sample_rate=self._model_params.sample_rate)
        elif self._model_params.mic_device.isdigit():
            return sr.Microphone(
                device_index=int(self._model_params.mic_device),
                sample_rate=self._model_params.sample_rate,
            )
        else:
            microphones = enumerate(sr.Microphone.list_microphone_names())
            for index, name in microphones:
                if self._model_params.mic_device in name:
                    return sr.Microphone(
                        device_index=index,
                        sample_rate=self._model_params.sample_rate,
                    )
            raise ValueError(
                f"Could not find microphone with name: {self._model_params.mic_device}"
            )

    def _configure_recogniser(
        self,
        energy_threshold: Optional[float] = None,
        pause_threshold: Optional[float] = None,
    ) -> sr.Recognizer:
        """Configures the speech recogniser object.

        Args:
            energy_threshold (float): Energy threshold for silence detection. Using this disables automatic adjustment.
            pause_threshold (float): Seconds of non-speaking audio before a phrase is considered complete.

        Returns:
            sr.Recognizer: speech recogniser object.
        """
        self._listening = True
        recogniser = sr.Recognizer()

        if pause_threshold:
            recogniser.pause_threshold = pause_threshold

        elif self._model_params.pause_threshold:
            recogniser.pause_threshold = self._model_params.pause_threshold

        if energy_threshold:
            recogniser.dynamic_energy_threshold = False
            recogniser.energy_threshold = energy_threshold
            return recogniser

        if self._model_params.energy_threshold:
            recogniser.dynamic_energy_threshold = False
            recogniser.energy_threshold = self._model_params.energy_threshold
            return recogniser

        with self._configure_microphone() as source:
            recogniser.adjust_for_ambient_noise(source)
        self._listening = False
        return recogniser

    def cancel_cb(self, goal_handle) -> CancelResponse:
        """Callback for cancelling the action server.
        Sets server to 'canceled' state.
        """
        cancel_str = f"{self._action_name} has been cancelled"
        self.get_logger().info(cancel_str)
        self._result.sequence = cancel_str

        # self._action_server.set_preempted(result=self._result, text=cancel_str)
        goal_handle.canceled()

        return CancelResponse.ACCEPT  # TODO decide if always accept cancellation

    async def execute_cb(self, goal_handle) -> None:
        """Callback for executing the action server.

        Checks for cancellation before listening and before and after transcribing, returning
        if cancellation is requested.

        Args:
            :param goal_handle: handles the goal request, and provides access to the goal parameters
        """

        goal = goal_handle.request

        self.get_logger().info("Request Received")
        if goal_handle.is_cancel_requested:
            return

        if goal.energy_threshold > 0.0 and goal.max_phrase_limit > 0.0:
            self.recogniser = self._configure_recogniser(
                goal.energy_threshold, goal.max_phrase_limit
            )
        elif goal.energy_threshold > 0.0:
            self.recogniser = self._configure_recogniser(goal.energy_threshold)
        elif goal.max_phrase_limit > 0.0:
            self.recogniser = self._configure_recogniser(
                pause_threshold=goal.max_phrase_limit
            )

        with self._configure_microphone() as src:
            self._listening = True
            wav_data = self.recogniser.listen(
                src,
                timeout=self._model_params.start_timeout,
                phrase_time_limit=self._model_params.phrase_duration,
            ).get_wav_data()
        # Magic number 32768.0 is the maximum value of a 16-bit signed integer
        float_data = (
            np.frombuffer(wav_data, dtype=np.int16).astype(np.float32, order="C")
            / 32768.0
        )

        if goal_handle.is_cancel_requested():
            self._listening = False
            self.get_logger().info("Goal was cancelled during execution.")
            goal_handle.canceled()
            return self._result

        self.get_logger().info(f"Transcribing phrase with Whisper...")
        transcription_start_time = timer()
        # Cast to fp16 if using GPU
        phrase = self._model.transcribe(
            float_data,
            fp16=self._model_params.device == "cuda",
        )["text"]
        transcription_end_time = timer()
        self.get_logger().info(f"Transcription finished!")
        self.get_logger().info(
            f"Time taken: {transcription_end_time - transcription_start_time:.2f}s"
        )
        self._transcription_server.publish(phrase)
        if goal_handle.is_cancel_requested():
            self._listening = False
            return

        self._result.sequence = phrase
        self.get_logger().info(f"Transcribed phrase: {phrase}")
        self.get_logger().info(f"{self._action_name} has succeeded")

        goal_handle.succeed()

        # Have this at the very end to not disrupt the action server
        self._listening = False

        return self._result


def parse_args() -> dict:
    """Parses the command line arguments into a name: value dictinoary.

    Returns:
        dict: Dictionary of name: value pairs of command line arguments.
    """
    parser = argparse.ArgumentParser(
        description="Starts an action server for transcribing speech."
    )

    parser.add_argument(
        "--action_name",
        type=str,
        default="transcribe_speech",
        help="Name of the action server.",
    )
    parser.add_argument(
        "--model_name",
        type=str,
        default="medium.en",
        help="Name of the speech recognition model.",
    )
    parser.add_argument(
        "--device",
        type=str,
        default="cuda" if torch.cuda.is_available() else "cpu",
        help="Device to run the model on.",
    )
    parser.add_argument(
        "--start_timeout",
        type=float,
        default=5.0,
        help="Timeout for listening for the start of a phrase.",
    )
    parser.add_argument(
        "--phrase_duration",
        type=float,
        default=10,
        help="Maximum phrase duration after starting listening in seconds.",
    )
    parser.add_argument(
        "--sample_rate",
        type=int,
        default=16000,
        help="Sample rate of the microphone.",
    )
    parser.add_argument(
        "--mic_device",
        type=str,
        default=None,
        help="Microphone device index or name",
    )
    parser.add_argument(
        "--no_warmup",
        action="store_true",
        help="Disable warming up the model by running inference on a test file.",
    )

    parser.add_argument(
        "--energy_threshold",
        type=Optional[int],
        default=None,
        help="Energy threshold for silence detection. Using this disables automatic adjustment",
    )

    parser.add_argument(
        "--pause_threshold",
        type=float,
        default=2.0,
        help="Seconds of non-speaking audio before a phrase is considered complete.",
    )

    args, unknown = parser.parse_known_args()
    return vars(args)


def configure_model_params(config: dict) -> speech_model_params:
    """Configures the speech model parameters based on the provided
    command line parameters.

    Args:
        config (dict): Command line parameters parsed in dictionary form.

    Returns:
        speech_model_params: dataclass containing the speech model parameters
    """
    model_params = speech_model_params()
    if config["model_name"]:
        model_params.model_name = config["model_name"]
    if config["device"]:
        model_params.device = config["device"]
    if config["start_timeout"]:
        model_params.start_timeout = config["start_timeout"]
    if config["phrase_duration"]:
        model_params.phrase_duration = config["phrase_duration"]
    if config["sample_rate"]:
        model_params.sample_rate = config["sample_rate"]
    if config["mic_device"]:
        model_params.mic_device = config["mic_device"]
    if config["no_warmup"]:
        model_params.warmup = False
    # if config["energy_threshold"]:
    #     model_params.energy_threshold = config["energy_threshold"]
    if config["pause_threshold"]:
        model_params.pause_threshold = config["pause_threshold"]

    return model_params


def configure_whisper_cache() -> None:
    """Configures the whisper cache directory."""
    whisper_cache = os.path.join(str(Path.home()), ".cache", "whisper")
    os.makedirs(whisper_cache, exist_ok=True)
    # Environmental variable required to run whisper locally
    os.environ["TIKTOKEN_CACHE_DIR"] = whisper_cache


def main(args=None):
    rclpy.init(args=args)

    configure_whisper_cache()
    config = parse_args()

    server = TranscribeSpeechAction("transcribe_speech", configure_model_params(config))

    try:
        rclpy.spin(server)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass