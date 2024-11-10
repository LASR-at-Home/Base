### Porting Interfaces to ROS2

- Interfaces are imported by their name, and have methods for request and response. 
- Examples:
  - Transcription service: Transcription.request(), Transcription.response()
  - TranscribeSpeech action: TranscribeSpeech.feedback(), TranscribeSpeech.Result()

#### Actions
- `actionlib` does not exist in ROS2.
- For `SimpleActionServer`, you can use `ActionServer` from `rclpy.action.server`
- See [preemption](PREEMPTION.md) documentation for how to handle goal cancellation.

#### Messages


#### Services