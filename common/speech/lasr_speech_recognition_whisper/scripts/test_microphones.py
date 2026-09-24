#!/usr/bin/env python3
"""Record a single utterance and save it as a WAV file for testing."""

import argparse
import numpy as np
import sounddevice as sd
import soundfile as sf

SAMPLE_RATE = 16000
DURATION = 10.0  # seconds


def parse_args():
    parser = argparse.ArgumentParser(description="Test microphone recording")
    parser.add_argument(
        "-m",
        "--microphone",
        type=str,
        default=None,
        help="Microphone name substring or index (default: system default)",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=str,
        default="/tmp/microphone_test.wav",
        help="Output WAV file path",
    )
    parser.add_argument(
        "-d",
        "--duration",
        type=float,
        default=DURATION,
        help="Recording duration in seconds",
    )
    args, _ = parser.parse_known_args()
    return args


def resolve_device(mic):
    if mic is None:
        return None
    if mic.isdigit():
        return int(mic)
    for idx, info in enumerate(sd.query_devices()):
        if mic in info["name"]:
            return idx
    raise ValueError(f"Could not find microphone: {mic}")


def main():
    args = parse_args()
    device = resolve_device(args.microphone)

    print(f"Recording {args.duration}s at {SAMPLE_RATE}Hz... speak now!")
    audio = sd.rec(
        int(args.duration * SAMPLE_RATE),
        samplerate=SAMPLE_RATE,
        channels=1,
        dtype="float32",
        device=device,
    )
    sd.wait()
    print("Done.")

    sf.write(args.output, audio, SAMPLE_RATE, subtype="PCM_16")
    print(f"Saved to {args.output}")


if __name__ == "__main__":
    main()
