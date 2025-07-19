#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import re
import smach

location_dict = {
    "table": "table",
    "sink": "sink",
    "shelf": "shelf",
    "fridge": "fridge",
    "cabinet": "cabinet",
    "dishwasher": "dishwasher",
    
}


class RecoverCommand(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["transcribed_speech"],
            output_keys=["location", "question"],
        )
        self.location_found = False

    def execute(self, userdata):
        sentence = re.sub(r'[^a-zA-Z\s]', '', userdata.transcribed_speech.lower())
        words = sentence.split()
        rospy.loginfo("Received sentence: %s", sentence)

        locations = []
        for word in words:
            rospy.loginfo("Processing word: %s", word)
            if word in location_dict:
                locations.append(location_dict[word])
                self.location_found = True

        if self.location_found:
            if len(locations) == 1:
                userdata.location = locations[0]
                return "succeeded"
            else:
                userdata.question = "I heard multiple locations. Please clarify."
                return "succeeded"
        else:
            userdata.location = None
            userdata.question = "I couldn't understand a location. Could you repeat?"
            return "succeeded"
        return "failed"



if __name__ == "__main__":
    rospy.init_node("recover_command_test")

    # Create an instance of your state
    state = RecoverCommand()

    # Set up dummy userdata
    class UserData(dict):
        pass

    userdata = UserData()
    userdata.transcribed_speech = "Can you put it on the table?"
    
    # Execute the state
    outcome = state.execute(userdata)

    # Print results
    print(f"\Outcome: {outcome}")
    if outcome == "succeeded":
        print(f"Detected location: {userdata.location}")
    else:
        print(f"Question: {userdata.question}")


