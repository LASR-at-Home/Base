#!/usr/bin/env python3
import smach
import rospy
import random

MOCK_ORDER = ["cup", "cup"]
class TakeOrder(smach.State):
    
    def __init__(self, head_controller, voice_controller, speech):
        smach.State.__init__(self, outcomes=['done'])
        self.head_controller = head_controller
        self.voice_controller = voice_controller
        self.speech = speech
        self.order = []

    def listen(self):
        resp = self.speech()
        if not resp:
            self.voice_controller.sync_tts("Sorry, I didn't get that")
            return self.listen()

    def get_order(self):
        resp = self.listen()
        item = [entity["item"]["value"] for entity in resp["entities"] if entity["entity"] == "item"][0]
        self.voice_controller.sync_tts(f"You asked for {item}, is that correct?")
        if not self.affirm():
            self.voice_controller.sync_tts("Okay, could you repeat please?")
            return self.get_order()
        return item

    def affirm(self):
        resp = self.listen()
        return bool([entity["choice"]["value"] for entity in resp["entities"] if entity["entity"] == "choice"][0])

    def execute(self, userdata):
        self.voice_controller.sync_tts("Can I please take your order?")
        order = []
        while True:
            order.append(self.get_order())
            self.voice_controller.sync_tts("Can I get you anything else?")
            if not self.affirm():
                break
        rospy.set_param(f"/tables/{rospy.get_param('/current_table')}/order", order)
        return 'done'
