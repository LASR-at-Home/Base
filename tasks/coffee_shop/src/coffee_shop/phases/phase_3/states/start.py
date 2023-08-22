import smach

class Start(smach.State):
    def __init__(self, voice_controller):
        smach.State.__init__(self, outcomes=['done'])
        self.voice_controller = voice_controller
    def execute(self, userdata):
        self.voice_controller.sync_tts("Starting Phase 3.")
        return 'done'
