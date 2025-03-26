### Porting Preemption to ROS2

Preemption has been removed, and cancellation has been introduced instead.

#### Cancelling an action server's goal
- To cancel an existing goal, use `CancelGoal` from `rclpy.action.server`
- When creating an `ActionServer`, pass a `cancel_callback` param with a method to handle cancelling.
- `self.cancel_cb` takes the goal handle as a parameter, and returns `CancelResponse.ACCEPT` (or REJECT)
- `server.set_preempted()` => `goal_handle.canceled()` 

See example in Whisper's [server](../common/speech/lasr_speech_recognition_whisper/lasr_speech_recognition_whisper/transcribe_microphone_server.py) node.

