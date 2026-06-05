import yasmin_ros
from lasr_speech_recognition_interfaces.action import TranscribeSpeech


class Listen(yasmin_ros.ActionState):
    def __init__(
        self
    ):
        super().__init__(
            action_name="transcribe_speech", 
            action_type=TranscribeSpeech, 
            result_handler=self.handle_resp,
            create_goal_handler=self.create_goal)
        
    def create_goal(self, blackboard):
        goal = TranscribeSpeech.Goal()
        return goal
        
    def handle_resp(self, blackboard, response):
        blackboard['sequence'] = response.sequence
        return 'succeeded'
