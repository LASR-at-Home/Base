import pyaudio

audio = pyaudio.PyAudio()
print("Available Audio Devices:\n")

for i in range(audio.get_device_count()):
    dev_info = audio.get_device_info_by_index(i)
    if dev_info["maxInputChannels"] > 0:  # Only look at recording devices
        print(f"Device ID {i}: {dev_info['name']}")
        print(f"  Default Sample Rate: {dev_info['defaultSampleRate']} Hz")

        # Test common native hardware sample rates
        for rate in [16000, 44100, 48000]:
            try:
                if audio.is_format_supported(
                    rate,
                    input_device=i,
                    input_channels=1,
                    input_format=pyaudio.paInt16,
                ):
                    print(f"  -> Supports: {rate} Hz")
            except ValueError:
                pass
audio.terminate()
