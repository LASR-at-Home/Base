#!/usr/bin/env python3

import rospy
from listen_module.subactions import BaseActions

class ButtonAction(BaseActions):
    def __init__(self, action_id):
        super(ButtonAction, self).__init__(action_id)

        self.subaction = {
            "confirm_button": self.confirm_button,
            "confirm_floor": self.ask_floor,
        }

    def ask_floor(self, audio_or_text):
        return self.handle_dialog_in_context(self.dialog_in_context(audio_or_text=audio_or_text, context="ConfirmFloor"),
                                             context="ConfirmFloor")

    def confirm_button(self, audio_or_text):
        return self.handle_dialog_in_context(
            self.dialog_in_context(audio_or_text=audio_or_text, context="ConfirmButton"), context="ConfirmButton")

    @staticmethod
    def handle_dialog_in_context(resp, context):
        try:
            # confirm button or floor
            if context == "ConfirmButton" or context == "ConfirmFloor":
                return resp.query_result.parameters["yesno"]
            else:
                rospy.loginfo("No context found in the navigation action")
                raise ValueError
        except ValueError:
            rospy.loginfo("I am not sure if tthe buttons are pressed and the floor is selected.")
            return -1
