> **Warning**: this package is not complete, this is subject to change.

List available microphones:

```bash
rosrun lasr_speech_recognition list_microphones.py
```

Start the example script:

```bash
rosrun lasr_speech_recognition transcribe_microphone <microphone_index>
```

Select microphone when prompted then wait for it to start.

You can now listen on `/transcription` for a live transcription.
