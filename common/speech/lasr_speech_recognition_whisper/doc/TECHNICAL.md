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
