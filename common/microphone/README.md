# microphone

Owns the robot's microphone and serves audio recordings to other nodes over a ROS service.

Only one process can reliably hold an audio input device at a time, so every node that needs audio
(speech recognition, doorbell detection, ...) goes through this node instead of opening the
microphone itself.

This package is maintained by:

- [Fadi Mostefai](mailto:fadimostefai@gmail.com)

## Prerequisites

This package depends on the following ROS packages:

- rclpy
- lasr_speech_recognition_interfaces (for the `RecordAudio` service)
- ament_virtualenv (build)

Python dependencies (see [requirements.in](requirements.in)):

- [sounddevice](https://pypi.org/project/sounddevice) – audio capture (requires PortAudio, `libportaudio2`)
- [silero-vad](https://pypi.org/project/silero-vad) – voice activity detection for phrase recording
- torch / torchaudio (CPU wheels are installed by `setup.py`)
- numpy

## Usage

Run the node:

```bash
ros2 run microphone mic
```

or in a launch file:

```python
microphone = Node(
    package="microphone",
    executable="mic",
    name="microphone_node",
    output="screen",
    parameters=[{"mic_device": "default"}],  # optional
)
```

Record a fixed amount of audio:

```bash
ros2 service call /microphone/record lasr_speech_recognition_interfaces/srv/RecordAudio \
  "{mode: 'fixed', duration: 2.0}"
```

Wait for someone to speak and record the phrase:

```bash
ros2 service call /microphone/record lasr_speech_recognition_interfaces/srv/RecordAudio \
  "{mode: 'phrase', start_timeout: 5.0, pause_threshold: 2.0}"
```

To see which input devices are available (for the `mic_device` parameter):

```bash
python3 -c "import sounddevice; print(sounddevice.query_devices())"
```

## Technical Overview

On startup the node opens a single `sounddevice.InputStream` (16 kHz, mono, float32, 512-sample
blocks) that stays open for the lifetime of the node. Audio chunks are only queued while a request is
being served; the queue is cleared at the start of each request so stale audio is never returned.

Requests are serialised with a lock, so concurrent callers wait their turn rather than receiving
interleaved audio.

The service supports two modes:

- **`fixed`** – records `duration` seconds and returns it. Used for continuous sound monitoring,
  e.g. the `DetectDoorbell` skill.
- **`phrase`** – uses [Silero VAD](https://github.com/snakers4/silero-vad) on each 512-sample chunk
  (speech if probability > 0.5):
  1. Waits up to `start_timeout` seconds for speech to start. If none is heard the request fails
     with `No speech detected`.
  2. Once speech starts, the previous ~0.5 s of audio (pre-roll) is included so the start of the
     first word isn't cut off.
  3. Recording continues until `pause_threshold` seconds of continuous silence, or until
     `max_phrase_duration` is reached.

If no audio chunk arrives for 0.5 s the stream is considered dead and the request fails with
`No audio received from the microphone`.

## ROS Definitions

### Launch Files

This package has no launch files. It is launched from task launch files, e.g.
`tasks/HRI/launch/HRI.launch.py` and `skills/launch/follow_person.launch.py`.

### Parameters

| Parameter             | Type   | Default     | Description                                                                                   |
|-----------------------|--------|-------------|-----------------------------------------------------------------------------------------------|
| `mic_device`          | string | `"default"` | Input device: an index (e.g. `"3"`), or a substring of the device name. Empty = system default. |
| `start_timeout`       | double | `5.0`       | Default seconds to wait for speech in `phrase` mode.                                          |
| `pause_threshold`     | double | `2.0`       | Default seconds of silence that end a phrase.                                                 |
| `max_phrase_duration` | double | `15.0`      | Maximum length of a recorded phrase, in seconds.                                              |

### Services

#### `/microphone/record` (`lasr_speech_recognition_interfaces/srv/RecordAudio`)

Request:

| Field             | Type    | Description                                                         |
|-------------------|---------|---------------------------------------------------------------------|
| `mode`            | string  | `"phrase"` or `"fixed"`.                                            |
| `duration`        | float32 | `fixed` mode: seconds to record (must be > 0).                      |
| `start_timeout`   | float32 | `phrase` mode: seconds to wait for speech (0 = node default).       |
| `pause_threshold` | float32 | `phrase` mode: seconds of silence that end the phrase (0 = node default). |

Response:

| Field         | Type      | Description                                                              |
|---------------|-----------|--------------------------------------------------------------------------|
| `success`     | bool      | False on invalid request, no speech before the timeout, or recording error. |
| `message`     | string    | Human-readable status or error.                                          |
| `samples`     | float32[] | Mono samples in [-1, 1].                                                 |
| `sample_rate` | uint32    | Always 16000.                                                            |

### Topics

This package publishes no topics.

### Actions

This package has no actions.
