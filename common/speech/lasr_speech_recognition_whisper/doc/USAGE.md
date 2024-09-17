> **Warning**: this package is not complete, this is subject to change.

List available microphones:

```bash
rosrun lasr_speech_recognition_whisper list_microphones.py
```

Run an actionlib server to transcribe the microphone:

```bash
rosrun lasr_speech_recognition_whisper transcribe_microphone_server
```

The response from the request is a `string` containing the transcribed text.

Several command line configuration options exist, which can be viewed with:

```bash
rosrun lasr_speech_recognition_whisper transcribe_microphone_server --help
```

Get tiago to repeat, with TTS the transcribed speech output; he will begin repeating after hearing "tiago, repeat ...." and stop once hearing "tiago, stop..."

```bash
rosrun lasr_speech_recognition_whisper repeat_after_me.py
```

To constantly listen and view transcribed speech output in the command line (by constantly sending requests to the actionlib server), run the following script:

```bash
rosrun lasr_speech_recongition_whisper test_speech_server.py
```


