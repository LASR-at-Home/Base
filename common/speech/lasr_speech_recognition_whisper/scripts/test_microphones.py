#!/usr/bin python3

import os
import argparse
import speech_recognition as sr
import rclpy

# TODO argparse -> ROS params

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

    # return vars(parser.parse_args())
    args, _ = parser.parse_known_args()
    return vars(args)


def main(args: dict = None) -> None:
    """Generate audio files from microphone input.

    Args:
        args (dict): dictionary of command line arguments.
    """

    # Adapted from https://github.com/Uberi/speech_recognition/blob/master/examples/write_audio.py

    rclpy.init(args=args)

    parser_args = parse_args()

    mic_index = parser_args["microphone"]
    output_dir = parser_args["output_dir"]

    r = sr.Recognizer()
    r.pause_threshold = 2
    with sr.Microphone(device_index=9, sample_rate=16000) as source:
        print("Say something!")
        audio = r.listen(source, timeout=5, phrase_time_limit=10)
        print("Finished listening")

    with open(os.path.join(output_dir, "microphone.raw"), "wb") as f:
        f.write(audio.get_raw_data())

    with open(os.path.join(output_dir, "microphone.wav"), "wb") as f:
        f.write(audio.get_wav_data())

    with open(os.path.join(output_dir, "microphone.flac"), "wb") as f:
        f.write(audio.get_flac_data())

    with open(os.path.join(output_dir, "microphone.aiff"), "wb") as f:
        f.write(audio.get_aiff_data())

    rclpy.shutdown()

if __name__ == "__main__":
    main()
