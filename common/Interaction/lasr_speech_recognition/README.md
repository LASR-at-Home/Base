# lasr_speech_recognition

The lasr_speech_recognition package

This package is maintained by:
- [Paul Makles](mailto:me@insrt.uk)

## Prerequisites

This package depends on the following ROS packages:
- catkin (buildtool)
- catkin_virtualenv (build)

This packages requires Python 3.10 to be present.

This package has 48 Python dependencies:
- [SpeechRecognition](https://pypi.org/project/SpeechRecognition)==3.10.0
- [openai-whisper](https://pypi.org/project/openai-whisper)==20230314
- [PyAudio](https://pypi.org/project/PyAudio)==0.2.13
- [PyYaml](https://pypi.org/project/PyYaml)==6.0.1
- [rospkg](https://pypi.org/project/rospkg)==1.5.0
- .. and 46 sub dependencies



## Usage



## Example



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
