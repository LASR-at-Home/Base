#!/usr/bin/env python3
import smach
import rospy
import random
import json
from collections import Counter

MOCK_ORDER = ["cup", "cup"]
class TakeOrder(smach.State):
    
    def __init__(self, head_controller, voice_controller, speech):
        smach.State.__init__(self, outcomes=['done'])
        self.head_controller = head_controller
        self.voice_controller = voice_controller
        self.speech = speech

    def listen(self):
        resp = self.speech()
        if not resp.success:
            self.voice_controller.sync_tts("Sorry, I didn't get that")
            return self.listen()
        return json.loads(resp.json_response)

    def get_order(self):
        resp = self.listen()
        if resp["intent"]["name"] != "make_order":
            self.voice_controller.sync_tts("Sorry, I didn't get that")
            return self.get_order()
        items = resp["entities"].get("item", [])
        if not items:
            self.voice_controller.sync_tts("Sorry, I didn't get that")
            return self.get_order()
        quantities = resp["entities"].get("quantity", [])
        quantified_items = []
        if len(items) == len(quantities) == 1:
            quantified_items.append((int(quantities[0]["value"]), items[0]["value"]))
        else:
            for item in items:
                quantified = False
                for quantity in quantities:
                    if quantity["end"] == item["start"] - 1:
                        quantified_items.append((int(quantity["value"]), item["value"]))
                        quantified = True
                        break
                if not quantified:
                    quantified_items.append((1, item["value"]))
        items = [[item]*quantity for quantity, item in quantified_items]
        items_string = ', '.join([f"{count} {item if count == 1 else item+'s'}" for item, count in Counter(order).items()]).replace(', ', ', and ', len(order)-2)

        self.voice_controller.sync_tts(f"You asked for {items_string}, is that correct?")
        if not self.affirm():
            self.voice_controller.sync_tts("Okay, could you repeat please?")
            return self.get_order()
        return items

    def affirm(self):
        resp = self.listen()
        if resp["intent"]["name"] != "affirm":
            self.voice_controller.sync_tts("Sorry, I didn't get that")
            return self.affirm()
        choices = resp["entities"].get("choice", None)
        if choices is None:
            self.voice_controller.sync_tts("Sorry, I didn't get that")
            return self.affirm()
        choice = choices[0]["value"]
        return bool(int(choice))

    def execute(self, userdata):
        self.voice_controller.sync_tts("Can I please take your order?")
        order = []
        while True:
            order.extend(self.get_order())
            self.voice_controller.sync_tts("Can I get you anything else?")
            if not self.affirm():
                break
            self.voice_controller.sync_tts("What else can I get for you?")
        self.voice_controller.sync_tts(f"Your order is {order}")
        rospy.set_param(f"/tables/{rospy.get_param('/current_table')}/order", order)
        return 'done'
