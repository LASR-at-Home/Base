# lasr_speech_recognition_interfaces

Common messages used for speech recognition

This package is maintained by:

- [Maayan Armony](mailto:maayan.armony@gmail.com)

## Prerequisites

This package depends on the following ROS packages:

- colcon (buildtool)
- message_generation (build)
- message_runtime (exec)

## Usage

Ask the package maintainer to write a `doc/USAGE.md` for their package!

## Example

Ask the package maintainer to write a `doc/EXAMPLE.md` for their package!

## Technical Overview

Ask the package maintainer to write a `doc/TECHNICAL.md` for their package!

## ROS Definitions

### Launch Files

This package has no launch files.

### Messages

#### `Transcription`

|  Field   |  Type  | Description |
|:--------:|:------:|-------------|
|  phrase  | string |             |
| finished |  bool  |             |

### Services

#### `RecordAudio`

Served by the `microphone` node on `/microphone/record`. See the
[microphone README](../../microphone/README.md) for details.

| Request field     | Type    | Description                                                              |
|-------------------|---------|--------------------------------------------------------------------------|
| `mode`            | string  | `"phrase"` (wait for speech, record until a pause) or `"fixed"`.         |
| `duration`        | float32 | `fixed` mode: seconds to record.                                         |
| `start_timeout`   | float32 | `phrase` mode: seconds to wait for speech (0 = node default).            |
| `pause_threshold` | float32 | `phrase` mode: seconds of silence that end the phrase (0 = node default). |

| Response field | Type      | Description                                                    |
|----------------|-----------|----------------------------------------------------------------|
| `success`      | bool      | False on invalid request, start timeout, or recording error.   |
| `message`      | string    | Status or error message.                                       |
| `samples`      | float32[] | Mono samples in [-1, 1].                                       |
| `sample_rate`  | uint32    | Sample rate of `samples` (16000).                              |

### Actions

#### `TranscribeSpeech`

Served by `lasr_speech_recognition_whisper` on `/transcribe_speech`.

| Goal field         | Type    | Description                                                     |
|--------------------|---------|-----------------------------------------------------------------|
| `energy_threshold` | float32 | Unused.                                                         |
| `max_phrase_limit` | float32 | Seconds of silence that end the phrase (0 = server default).    |

| Result field | Type   | Description                                      |
|--------------|--------|--------------------------------------------------|
| `sequence`   | string | Transcribed phrase, or `""` if nothing was heard. |
