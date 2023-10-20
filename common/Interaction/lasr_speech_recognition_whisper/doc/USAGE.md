> **Warning**: this package is not complete, this is subject to change.

List available microphones:

```bash
rosrun lasr_speech_recognition_whisper list_microphones.py
```

Start the example script:

```bash
rosrun lasr_speech_recognition_whisper transcribe_microphone by-index <microphone_index>
rosrun lasr_speech_recognition_whisper transcribe_microphone by-name <substring_of_name>
```

Then start listening to people:

```bash
rosservice call /whisper/start_listening "{}"
```

You can now listen on `/transcription` for a live transcription.

Stop listening whenever:

```bash
rosservice call /whisper/stop_listening "{}"
```
