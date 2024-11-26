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
- Service nodes in ROS2 should inherit the `Node` class which is imported from `rclpy.node`.
-  The service class' constructor should initialise the node with its name.
- The service should then be created in the constructor with the create_service function, taking in the type, name, and callback.