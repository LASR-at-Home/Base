#!/usr/bin/env python3
import smach
import rospy
import json
from collections import Counter
import difflib
from play_motion_msgs.msg import PlayMotionGoal
from control_msgs.msg import PointHeadGoal
from geometry_msgs.msg import Point
from coffee_shop_ui.msg import Order
from std_msgs.msg import String

class TakeOrder(smach.State):
    
    def __init__(self, context):
        smach.State.__init__(self, outcomes=['done'])
        self.context = context
        self.tablet_pub = rospy.Publisher("/tablet/screen", String, queue_size=10) 

    def listen(self):
        resp = self.context.speech(True)
        if not resp.success:
            self.context.voice_controller.sync_tts(self.context.get_random_retry_utterance())
            return self.listen()
        resp = json.loads(resp.json_response)
        rospy.loginfo(resp)
        return resp

    def get_order(self):
        resp = self.listen()
        if resp["intent"]["name"] != "make_order":
            rospy.logwarn("The intent was wrong")
            self.context.voice_controller.sync_tts(self.context.get_random_retry_utterance())
            return self.get_order()
        items = resp["entities"].get("item", [])
        if not items:
            rospy.logwarn("There were no items")
            self.context.voice_controller.sync_tts(self.context.get_random_retry_utterance())
            return self.get_order()
        quantities = resp["entities"].get("CARDINAL", [])
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
        items = []
        for quantity, item in quantified_items:
            if item not in self.context.target_object_remappings.keys():
                matches = difflib.get_close_matches(item.lower(), self.context.target_object_remappings.keys())
                if matches:
                    item = matches[0]
                else:
                    continue
            items.extend([item.lower()]*quantity)
        if not items:
            self.context.voice_controller.sync_tts(self.context.get_random_retry_utterance())
            return self.get_order()
        return items

    def affirm(self):
        resp = self.listen()
        if resp["intent"]["name"] not in ["affirm", "deny"]:
            self.context.voice_controller.sync_tts(self.context.get_random_retry_utterance())
            return self.affirm()
        return resp["intent"]["name"] == "affirm"

    def execute(self, userdata):
        self.context.stop_head_manager("head_manager")

        if self.context.tablet:
            self.context.voice_controller.sync_tts("Hello, I'm TIAGo, I'll be serving you today. Please use the tablet to make your order.")  
            pm_goal = PlayMotionGoal(motion_name="tablet", skip_planning=True)
            self.context.play_motion_client.send_goal_and_wait(pm_goal)
            self.tablet_pub.publish(String("order"))
            order = rospy.wait_for_message("/tablet/order", Order).products 
        else:
            ph_goal = PointHeadGoal()
            ph_goal.max_velocity = 1.0
            ph_goal.pointing_frame = 'head_2_link'
            ph_goal.pointing_axis = Point(1.0, 0.0, 0.0)
            ph_goal.target.header.frame_id = 'map'
            ph_goal.target.point = Point(*self.context.get_interaction_person())
            pm_goal = PlayMotionGoal(motion_name="back_to_default", skip_planning=True)
            self.context.play_motion_client.send_goal_and_wait(pm_goal)

            if len(self.context.tables[self.context.current_table]["people"]) == 1:
                self.context.point_head_client.send_goal_and_wait(ph_goal)
                self.context.voice_controller.sync_tts("Hello, I'm TIAGo, I'll be serving you today.")
                self.context.voice_controller.sync_tts("Please state your order after the beep - this indicates that I am listening.")
            elif len(self.context.tables[self.context.current_table]["people"]) == 2:
                self.context.voice_controller.sync_tts("Greetings to both of you, I'm TIAGo, I'll be serving you today.")
                self.context.point_head_client.send_goal_and_wait(ph_goal)
                self.context.voice_controller.sync_tts("I choose you to be the one in charge.")
                self.context.voice_controller.sync_tts("Please state the order for the two of you after the beep - this indicates that I am listening.")
            else:
                self.context.voice_controller.sync_tts("Salutations to all of you, I'm TIAGo, I'll be serving you today.")
                self.context.point_head_client.send_goal_and_wait(ph_goal)
                self.context.voice_controller.sync_tts("I choose you to be the one in charge.")
                self.context.voice_controller.sync_tts("Please state the order for the group after the beep - this indicates that I am listening.")

            order = []

            while True:
                order.extend(self.get_order())

                items_string = ', '.join([f"{count} {self.context.target_object_remappings[item] if count == 1 else self.context.target_object_remappings[item]+'s'}" for item, count in Counter(order).items()]).replace(', ', ', and ', len(order)-2)

                self.context.voice_controller.sync_tts(f"You asked for {items_string} so far, can I get you anything else? Please answer yes or no after the beep.")
                if self.affirm():
                    self.context.voice_controller.sync_tts("Okay, please state the additional items after the beep.")
                else:
                    break
        order_string = ', '.join([f"{count} {self.context.target_object_remappings[item] if count == 1 else self.context.target_object_remappings[item]+'s'}" for item, count in Counter(order).items()]).replace(', ', ', and ', len(order)-2)

        self.context.voice_controller.sync_tts(f"Your order is {order_string}")
        self.context.tables[self.context.current_table]["order"] = order
        pm_goal = PlayMotionGoal(motion_name="back_to_default", skip_planning=True)
        self.context.play_motion_client.send_goal_and_wait(pm_goal)
        self.context.start_head_manager("head_manager", '')
        return 'done'
