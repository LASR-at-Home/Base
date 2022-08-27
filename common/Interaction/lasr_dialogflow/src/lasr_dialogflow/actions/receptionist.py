#!/usr/bin/env python3
import random
import rospy
from lasr_dialogflow.actions import BaseAction
from std_msgs.msg import String
from google.api_core.exceptions import DeadlineExceeded

class ReceptionistAction(BaseAction):
    def __init__(self, project_id, df_lang_id="en"):
        super(ReceptionistAction, self).__init__(project_id, df_lang_id="en")

        self.name = None
        self.name_confirmed = None
        self.current_context = None
        self.favourite_drink = None
        self.max_retries = 3

        self.actions = {
            "ask_name" : self.ask_name,
            "ask_favourite_drink" : self.ask_favourite_drink,
            "stop" : self.stop,
        }

    def ask_name(self, use_mic=False, text=None):

        if not use_mic and text:
            print(self.handle_response(self.text_in_context(text, context="GetName"), "GetName"))
            return self.handle_response(self.text_in_context(text, context="GetName"), "GetName")
        elif use_mic:
            self.say_text("What's your name?")
            attempt = 0
            while attempt < self.max_retries:
                name = self.handle_response(self.listen_in_context("GetName"), "GetName")
                if name:
                    self.say_text(f"Your name is {name}, is that correct?")
                    correct = self.handle_response(self.listen_in_context("ConfirmCheck"), "ConfirmCheck")
                    if correct == "yes":
                        return name
                    else:
                        self.say_text("Okay. Can you repeat your name please?")
                else:
                    self.say_text("Sorry, I didn't get that. Can you repeat please?")
                attempt +=1
        
        return ""

    def ask_favourite_drink(self):
        self.say_text("What's your favourite drink?")

        attempt = 0
        while attempt < self.max_retries:
            drink = self.handle_response(self.listen_in_context("GetFavouriteDrink"), "GetFavouriteDrink")
            if drink:
                self.say_text(f"Your favourite drink is {drink}, is that correct?")
                correct = self.handle_response(self.listen_in_context("ConfirmCheck"), "ConfirmCheck")
                if correct == "yes":
                    return drink
                else:
                    self.say_text("Okay. Can you repeat your favourite drink please?")
            else:
                self.say_text("Sorry, I didn't get that. Can you repeat please?")
            attempt +=1
        
        return ""

    def handle_response(self, response, context):
        if not response:
            return None
        try:
            if context == "GetName":
                return response.query_result.parameters["given-name"]
            elif context == "GetFavouriteDrink":
                print(response.query_result)
                return response.query_result.parameters["drink-name"]
            elif context == "ConfirmCheck":
                return response.query_result.parameters["yesno"]
        except ValueError:
            return None