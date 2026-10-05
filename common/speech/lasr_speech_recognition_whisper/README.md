# lasr_speech_recognition_whisper

Speech recognition implemented using OpenAI Whisper

This package is maintained by:

- [Maayan Armony](mailto:maayan.armony@gmail.com)

## Prerequisites

This package depends on the following ROS packages:

- colcon (buildtool)
- lasr_speech_recognition_interfaces
- [microphone](../../microphone/README.md) (at runtime – provides the `/microphone/record` service)

Python dependencies (see [requirements file](requirements.txt)):

- [openai-whisper](https://pypi.org/project/openai-whisper)
- [soundfile](https://pypi.org/project/soundfile) – saving recordings
- [SpeechRecognition](https://pypi.org/project/SpeechRecognition), [PyAudio](https://pypi.org/project/PyAudio), [sounddevice](https://pypi.org/project/sounddevice) – used by the helper scripts
- .. and sub dependencies

This package requires that [ffmpeg](https://ffmpeg.org/) is available during runtime.

A CUDA GPU is used automatically if available; otherwise Whisper runs on CPU.

## Usage

The transcription server does **not** open the microphone itself. It requests audio from the
`microphone` node, so both must be running:

```bash
ros2 run microphone mic
ros2 run lasr_speech_recognition_whisper transcribe_microphone_server
```

The server waits for `/microphone/record` to become available before it starts its action server.

Or, in a launch file:

```python
microphone = Node(
    package="microphone",
    executable="mic",
    name="microphone_node",
    output="screen",
)

transcribe_speech = Node(
    package="lasr_speech_recognition_whisper",
    executable="transcribe_microphone_server",
    name="whisper_mic_server",
    output="screen",
    parameters=[{"model": "medium.en"}],  # optional
)
```

To transcribe one phrase:

- In a terminal

    ```bash
    ros2 action send_goal /transcribe_speech lasr_speech_recognition_interfaces/action/TranscribeSpeech "{max_phrase_limit: 0.0}"
    ```

- With the test client

    ```bash
    ros2 run lasr_speech_recognition_whisper test_speech_server
    ```

- From a state machine, use the `Listen` or `AskAndListen` skills (`lasr_skills`).

The transcription is also published on `/live_speech_transcription`.

### Helper scripts

| Executable               | Description                                     |
|--------------------------|-------------------------------------------------|
| `test_speech_server`     | Sends goals to `/transcribe_speech` and prints the results. |
| `list_microphones`       | Lists audio input devices.                      |
| `test_microphones`       | Records ~10 s from a microphone and saves it as a WAV (`-m <name or index>`, `-o <path>`). |
| `microphone_tuning_test` | Repeatedly transcribes speech with `medium.en`, raising the energy threshold each time, to tune the microphone (`--device_index <n>`). |

> **Note**: `list_microphones`, `test_microphones` and `microphone_tuning_test` open the microphone
> directly, so stop the `microphone` node before running them.

## Technical Overview

Each `TranscribeSpeech` goal is handled as follows:

1. **Record** – the server calls `/microphone/record` in `phrase` mode. The microphone node uses
   Silero VAD to wait for speech (up to `start_timeout`) and records until `pause_threshold` seconds
   of silence. See the [microphone README](../../microphone/README.md) for details.
2. **Transcribe** – the returned 16 kHz float samples are passed straight to the local Whisper
   model, with decoding options (`no_speech_threshold`, `logprob_threshold`,
   `hallucination_silence_threshold`, ...) set to reduce hallucinations on noise.
3. **Filter** – common Whisper hallucinations on near-silent audio (`""`, `"you"`, `"thank you."`,
   `"thanks."`, `"."`) are replaced with an empty string.
4. **Return** – the phrase is published on `/live_speech_transcription` and returned as the action
   result. If `save_audio` is enabled, the audio and transcript are written to `save_audio_dir` as
   `<timestamp>.wav` / `<timestamp>.txt`.

The model is loaded and warmed up with one second of silence at startup, so the first real request
isn't slowed down.

Behaviour on failure:

| Situation                                       | Goal status | `sequence` |
|-------------------------------------------------|-------------|------------|
| Nobody spoke before the start timeout           | succeeded   | `""`       |
| Hallucination filtered                          | succeeded   | `""`       |
| Microphone service error / Whisper error        | aborted     | `""`       |
| Goal cancelled                                  | canceled    | `""`       |

A recording in progress on the microphone node can't be interrupted; on cancel the server stops
waiting and discards the recording when it arrives.

## ROS Definitions

### Launch Files

This package has no launch files.

### Parameters (`transcribe_microphone_server`)

| Parameter                         | Type     | Default                      | Description                                                          |
|-----------------------------------|----------|------------------------------|----------------------------------------------------------------------|
| `model`                           | string   | `"small.en"`                 | Whisper model name.                                                  |
| `device`                          | string   | `"cuda"` if available, else `"cpu"` | Device to run Whisper on.                                     |
| `start_timeout`                   | double   | `5.0`                        | Seconds to wait for speech to start.                                 |
| `pause_threshold`                 | double   | `2.0`                        | Seconds of silence that end a phrase (overridden by `max_phrase_limit` in the goal). |
| `condition_on_previous_text`      | bool     | `false`                      | Whisper decoding option.                                             |
| `initial_prompt`                  | string   | `""`                         | Text to bias the transcription, e.g. expected names or keywords.     |
| `no_speech_threshold`             | double   | `0.4`                        | Drops segments Whisper thinks are not speech.                        |
| `logprob_threshold`               | double   | `-0.8`                       | Drops low-confidence segments.                                       |
| `temperature`                     | double[] | `[0.0, 0.2]`                 | Temperatures to retry decoding at.                                   |
| `word_timestamps`                 | bool     | `true`                       | Required for `hallucination_silence_threshold`.                      |
| `hallucination_silence_threshold` | double   | `0.5`                        | Skips silent periods longer than this (seconds).                     |
| `save_audio`                      | bool     | `true`                       | Save each recording and its transcript.                              |
| `save_audio_dir`                  | string   | `"/tmp/whisper_recordings"`  | Where recordings are saved.                                          |

### Topics

| Topic                        | Type              | Description                          |
|------------------------------|-------------------|--------------------------------------|
| `/live_speech_transcription` | `std_msgs/String` | Published once per transcribed goal. |

### Services

This package provides no services. It is a client of `/microphone/record`
(`lasr_speech_recognition_interfaces/srv/RecordAudio`).

### Actions

#### `/transcribe_speech` (`lasr_speech_recognition_interfaces/action/TranscribeSpeech`)

Goal:

| Field              | Type    | Description                                                              |
|--------------------|---------|--------------------------------------------------------------------------|
| `energy_threshold` | float32 | Unused (kept for compatibility).                                         |
| `max_phrase_limit` | float32 | Seconds of silence that end the phrase. `0` uses the `pause_threshold` parameter. |

Result:

| Field      | Type   | Description                                   |
|------------|--------|-----------------------------------------------|
| `sequence` | string | The transcribed phrase, or `""` if nothing was heard. |
