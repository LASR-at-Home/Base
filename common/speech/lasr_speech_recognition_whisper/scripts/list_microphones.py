#!/usr/bin python3
import speech_recognition as sr
import sounddevice  # needed to remove ALSA error messages


def main():
    microphones = enumerate(sr.Microphone.list_microphone_names())

    print("\nAvailable microphones:")
    for index, name in microphones:
        print(f"[{index}] {name}")

    # # Uncomment for debugging, to see if sounddevice recongises the microphone as well
    # print("Available microphone devices (sounddevice):")
    # print(sounddevice.query_devices())


if __name__ == "__main__":
    main()
