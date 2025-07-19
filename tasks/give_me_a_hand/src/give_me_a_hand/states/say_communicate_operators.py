#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import re

location_dict = {
                "dishwasher": "dishwasher",
                "automatic dishwasher": "dishwasher"
                "sink":"sink",
                "table":"table",
                "fridge":"fridge",
                "shelf":"shelf",
            }

class SayCommunicateOperators():
    def __init__(
        self,
    ):
        smach.StateMachine.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_key=["sentence"],
            output_key=["location"])
    
    def execute(self, userdata):
        sentence = userdata.sentence

        sentence = re.sub(r'[^a-zA-Z\s]', '', sentence)
        
        words = sentence.split()
        rospy.loginfo("Received sentence: %s", sentence)

        for word in location_dict:
            value = location.get(word)

            if value is not None:
                print("Found:", value)
            else:
                print("Not found")

            
        

# # did two or more location been mentioned?

# direction = {
#     "front": "",
#     "behind": "",
#     "next": "",
#     "up": "",
#     "down": "",
#     "rear": "",
#     "back": "",
#     "right": "",
#     "left": "",
#     "side": "",
#     "upper side": "",
#     "lower side": "",
#     "outside": "",
#     "inside": "",
#     "outer": "",
#     "inner": "",
#     "west": "",
#     "east": "",
#     "south": "",
#     "north": ""
# }

# # How many times?

# @

# #& multiple

#1. nothing (put it, place it)
f"Need more information. please tell the location like dishwasher."
# f"dishwasher? sink? fridge? shelf? table?"

#1. only direction (place behind, behind plz)
f"{direction} of what or where? Could you explain more?"

#1. only object (place brush, put cup)
f"Where shell I place ?"

#1. only demonstrative_pronoun (place there, place here)
f"{demonstrative_pronoun}? Please explain more."

#1. only location ()
f"I'll put at {location}. Am I right?" #shell I not check?

#2. direction and object (on the dish, put dish up)
f"Dish on where?"

#2. direction and demonstrative_pronoun (up there, down here)
f"{demonstrative_pronoun} where? Shelf? Table? explain more."

#2. direction and location (inside of the shelf, shelf inside)
f"I'll put at {location}. Am I right?"

#2. direction and Trick words ()

#2. 

# Trick words?
{

}

f"while I'm saying you can always interupted. Just say Hi, tiago." #interupped 
#concurrent state

{there here }

object = {
    "brush":"",
    "cloth":"",
    "polish":"",
    "sponge":"",
    "bowl":"",
    "cup":"",
    "fork":"",
    "knife":"",
    "plate":"",
    "spoon":"",
    "coffee":"",
    "coke":"",
    "fanta":"",
    "kuat":"",
    "milk":"",
    "orange juice":"",
    "broth":"",
    "broth box":"",
    "corn flour":"",
    "ketchup":"",
    "mayo":"",
    "oats":"",
    "tuna":"",
    "apple":"",
    "lemon":"",
    "lime":"",
    "pear":"",
    "tangerine":"",
    "cheese snack":"",
    "chocolate bar":"",
    "cornflakes":"",
    "crisps":"",
    "gum balls":"",
    "peanuts":"",
    "pringles":"",
    "bag":"",
    "dishwasher tab":"",
    "dishwasher tab_bag":"",
    "cornflakes container":"",
}

direction = {
    "up": "",
    "down": "",
    "left": "",
    "right": "",
    "front": "",
    "back": "",
    "behind": "",
    "rear": "",
    "top": "",
    "bottom": "",
    "side": "",
    "center": "",
    "middle": "",
    "inside": "",
    "outside": "",
    "inner": "",
    "outer": "",
    "upper": "",
    "lower": "",
    "above": "",
    "below": "",
    "beneath": "",
    "under": "",
    "underneath": "",
    "over": "",
    "across": "",
    "through": "",
    "in": "",
    "inside":"",
    "outside":"",
    "out": "",
    "forward": "",
    "backward": "",
    "clockwise": "",
    "counterclockwise": "",
    "vertical": "",
    "horizontal": "",
    "diagonal": "",
    "north": "",
    "south": "",
    "east": "",
    "west": "",
    "northeast": "",
    "northwest": "",
    "southeast": "",
    "southwest": "",
    "inward": "",
    "outward": "",
    "toward": "",
    "away": "",
    "adjacent": "",
    "proximal": "",
    "distal": "",
    "dorsal": "",
    "ventral": "",
    "lateral": "",
    "medial": "",
    "superior": "",
    "inferior": "",
    "anterior": "",
    "posterior": "",
    "above":"",
}

demonstrative_pronoun = {
    "here",        
    "there",      
    "nearby",      # close but not exact
    "far",         # distant
    "away",        # moving farther
    "along",       # in a direction or path
    "around",      # encircling, nearby
    "near",        # close proximity
    "next to",     # adjacent
    "beside",      # at the side
    "beyond",      # farther than something
    "within",      # inside bounds
    "without",     # outside bounds
    "ahead",       # in front
    "behind",      # to the rear
    "off",         # not on / disconnected
    "on",          # on top of or touching
    "in",          # inside
    "out",         # outside
    "upstairs",    # on a higher floor
    "downstairs",  # on a lower floor
    "indoors",     # inside a building
    "outdoors"     # outside a building
}
