# lasr_speech_recognition_whisper

Speech recognition implemented using OpenAI Whisper

This package is maintained by:
- [Paul Makles](mailto:me@insrt.uk)

## Prerequisites

This package depends on the following ROS packages:
- catkin (buildtool)
- catkin_virtualenv (build)
- lasr_speech_recognition_msgs

This packages requires Python 3.10 to be present.

This package has 48 Python dependencies:
- [SpeechRecognition](https://pypi.org/project/SpeechRecognition)==3.10.0
- [openai-whisper](https://pypi.org/project/openai-whisper)==20230314
- [PyAudio](https://pypi.org/project/PyAudio)==0.2.13
- [PyYaml](https://pypi.org/project/PyYaml)==6.0.1
- [rospkg](https://pypi.org/project/rospkg)==1.5.0
- .. and 46 sub dependencies

This package requires that [ffmpeg](https://ffmpeg.org/) is available during runtime.

## Usage

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

## Example

Ask the package maintainer to write a `doc/EXAMPLE.md` for their package!

## Technical Overview

This package does speech recognition in three parts:

- Adjusting for background noise

  We wait for a set period of time monitoring the audio stream to determine what we should ignore when collecting voice data.

- Collecting appropriate voice data for phrases

  We use the `SpeechRecognition` package to monitor the input audio stream and determine when a person is actually speaking with enough energy that we would consider them to be speaking to the robot.

- Running inference on phrases

  We continuously combine segments of the spoken phrase to form a sample until a certain timeout or threshold after which the phrase ends. This sample is sent to a local OpenAI Whisper model to transcribe.

The package can input from the following sources:

- On-board or external microphone on device
- Audio data from ROS topic (WORK IN PROGRESS)

The package can output transcriptions to:

- Standard output
- A ROS topic

## ROS Definitions

### Launch Files

This package has no launch files.

### Messages

This package has no messages.

### Services

This package has no services.

### Actions

This package has no actions.
