#!/usr/bin/env python3

import rospy
from listen_module.subactions import BaseActions


class NavigationAction(BaseActions):
    def __init__(self, action_id):
        super(NavigationAction, self).__init__(action_id)

        self.subaction = {
            "confirm_location": self.confirm_location,
            "ask_location": self.ask_location,
        }

    def ask_location(self, audio_or_text):
        return self.handle_dialog_in_context(self.dialog_in_context(audio_or_text=audio_or_text, context="AskLocation"),
                                             context="AskLocation")

    def confirm_location(self, audio_or_text):
        return self.handle_dialog_in_context(
            self.dialog_in_context(audio_or_text=audio_or_text, context="ConfirmLocation"), context="ConfirmLocation")

    @staticmethod
    def handle_dialog_in_context(resp, context):
        try:
            # number, person, Destination
            if context == "ConfirmLocation":
                return resp.query_result.parameters["yesno"]
            elif context == "AskLocation":
                print(resp.query_result)
                return resp.query_result.parameters["Destination"] or resp.query_result.parameters["number"] or \
                       resp.query_result.parameters["person"]
            else:
                rospy.loginfo("No context found in the navigation action")
                raise ValueError
        except ValueError:
            print("I don't know where you want to go.")
