#!/usr/bin/env python3
import speech_recognition as sr
microphones = enumerate(sr.Microphone.list_microphone_names())

print('\nAvailable microphones:')
for index, name in microphones:
    print(f"[{index}] {name}")
