#!/usr/bin/env python3

import os
import argparse
import speech_recognition as sr


def parse_args() -> dict:
    """Parse command line arguments into a dictionary.

    Returns:
        dict: name: value pairs of command line arguments
    """

    parser = argparse.ArgumentParser(description="Test microphones")
    parser.add_argument("-m", "--microphone", type=int, help="Microphone index")
    parser.add_argument(
        "-o", "--output_dir", type=str, help="Directory to save audio files"
    )

    return vars(parser.parse_args())


def main(args: dict) -> None:
    """Generate audio files from microphone input.

    Args:
        args (dict): dictionary of command line arguments.
    """

    # Adapted from https://github.com/Uberi/speech_recognition/blob/master/examples/write_audio.py

    mic_index = args["microphone"]
    output_dir = args["output_dir"]

    r = sr.Recognizer()
    with sr.Microphone(device_index=13,sample_rate=16000) as source:
        print("Say something!")
        audio = r.listen(source, timeout=5, phrase_time_limit=5)
        print("Finished listening")

    with open(os.path.join(output_dir, "microphone.raw"), "wb") as f:
        f.write(audio.get_raw_data())

    with open(os.path.join(output_dir, "microphone.wav"), "wb") as f:
        f.write(audio.get_wav_data())

    with open(os.path.join(output_dir, "microphone.flac"), "wb") as f:
        f.write(audio.get_flac_data())

    with open(os.path.join(output_dir, "microphone.aiff"), "wb") as f:
        f.write(audio.get_aiff_data())


if __name__ == "__main__":
    main(parse_args())
