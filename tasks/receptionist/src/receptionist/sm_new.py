import smach 
from receptionist.phases import Phase1, Phase2, Phase3 
from receptionist.default import Default 



class Receptionist(smach.StateMachine):
    def __init__(self):
        self.default = Default()
        self.userdata.area_polygon = [[1.94, 0.15], [2.98, 0.28], [3.08, -0.68], [2.06, -0.84]]
        self.userdata.depth_topic = "/xtion/depth_registered/points"

        with self: 
            smach.StateMachine.add('PHASE1', Phase1(self.default), transitions={'success' : 'PHASE2'})
            smach.StateMachine.add('PHASE2', Phase2(self.default), transitions={'success' : 'PHASE3'})
            smach.StateMachine.add('PHASE3', Phase3(self.default), transitions={'success' : 'success'})


