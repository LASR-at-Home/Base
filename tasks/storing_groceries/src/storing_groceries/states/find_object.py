import smach

from lasr_skills import PlayMotion


class FindObject(smach.StateMachine):

    def __init__(self):
        super().__init__(self, outcomes=["succeeded", "failed"])

        with self:
            smach.StateMachine.add("")
