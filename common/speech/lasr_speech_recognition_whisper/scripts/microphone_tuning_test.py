#!/usr/bin python3
import argparse
import os
import torch
import numpy as np
from pathlib import Path
import speech_recognition as sr
from lasr_speech_recognition_whisper import load_model  # type: ignore
import sounddevice  # needed to remove ALSA error messages
from typing import Dict


def parse_args() -> Dict:
    parser = argparse.ArgumentParser()
    parser.add_argument("--device_index", type=int, default=None)
    return vars(parser.parse_args())


def configure_whisper_cache() -> None:
    """Configures the whisper cache directory."""
    whisper_cache = os.path.join(str(Path.home()), ".cache", "whisper")
    os.makedirs(whisper_cache, exist_ok=True)
    # Environemntal variable required to run whisper locally
    os.environ["TIKTOKEN_CACHE_DIR"] = whisper_cache


def main():
    args = parse_args()

    recognizer = sr.Recognizer()
    recognizer.pause_threshold = 2
    microphone = sr.Microphone(device_index=args["device_index"], sample_rate=16000)
    threshold = 100
    recognizer.dynamic_energy_threshold = False
    recognizer.energy_threshold = threshold
    transcription_model = load_model(
        "medium.en", "cuda" if torch.cuda.is_available() else "cpu", True
    )
    transcription_result = "The quick brown fox jumps over the lazy dog."
    while transcription_result != "":
        print(f"Listening...")
        with microphone as source:
            wav_data = recognizer.listen(
                source, phrase_time_limit=10, timeout=5
            ).get_wav_data()
        print(f"Processing...")
        # Magic number 32768.0 is the maximum value of a 16-bit signed integer
        float_data = (
            np.frombuffer(wav_data, dtype=np.int16).astype(np.float32, order="C")
            / 32768.0
        )

        # Cast to fp16 if using GPU
        transcription_result = transcription_model.transcribe(
            float_data, fp16=torch.cuda.is_available()
        )["text"]

        print(
            f"Transcription: {transcription_result} at energy threshold {recognizer.energy_threshold}"
        )
        threshold += 100
        recognizer.energy_threshold = threshold


if __name__ == "__main__":
    configure_whisper_cache()
    main()
