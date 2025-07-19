#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import re
import smach

class CommunicateOperators(smach.State):
    def __init__(
        self,
    ):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["transcribed_speech"],
            output_keys=["location"])

            self.trick_words_found = False
            self.location_found = False
            self.object_found = False
            self.direction_found = False
            self.demonstrative_pronoun_found = False
    
    def execute(self, userdata):
        sentence = re.sub(r'[^a-zA-Z\s]', '', userdata.transcribed_speech.lower())
        words = sentence.split()
        rospy.loginfo("Received sentence: %s", sentence)

        for word in words:
            rospy.loginfo("Processing word: %s", word)
            trick_words_value = trick_words_dict.get(word)
            location_value = location_dict.get(word)
            object_value = object_dict.get(word)
            direction_value = direction_dict.get(word)
            demonstrative_pronoun_found = demonstrative_pronoun_dict.get(word)

            trick_words = []
            location_value = []
            object_value = []
            direction_value = []
            demonstrative_pronoun_value = []

            if trick_words_value is not None:
                print("Found:", trick_words_value)
                trick_words.append(trick_words_value)
                self.trick_words_found = True
            if location_value is not None:
                print("Found:", location_value)
                locations.append(location_value)
                self.location_found = Ture
            if object_value is not None:
                print("Found:", object_value)
                objects.append(object_value)
                self.object_found = False
            if direction_value is not None:
                print("Found:", direction_value)
                detection.append(direction_value)
                self.direction_found = False
            if  demonstrative_pronoun_value is not None:
                print("Found:", demonstrative_pronoun_value)
                demonstrative_pronouns.append(demonstrative_pronoun_value)
                self.demonstrative_pronoun_found = False

            if trick_words_found:
                if location_found:
                    if "but" in trick_words:
                        idx = locations.index("but")
                        print("Found at index:", idx)
                        userdata.location = demonstrative_pronoun[idx]
                        return "succeeded"
                else:
                    if object_found:
                        ""
                    else:
                        if direction_found:
                        else:
                            if demonstrative_pronoun_found:
                                if demonstrative_pronoun == :
                                    if "but" in trick_words:
                                        idx = locations.index("but")
                                        print("Found at index:", idx)
                                        userdata.location = demonstrative_pronoun[idx]
                                        return "succeeded"
                                    # if "instead"
                                    # elif "not":
                                    # elif "rather":
                                    # elif "morethen":
                                    
                                    # trick_words_dict = {
                                    #     "not": "",
                                    #     "but": ""
                                    #     "rather":"",
                                    #     "morethen":"",
                                    # }    

                            
                            else:
                                userdata.question = "Please repeat with easier sentence."
                                return "failed",

            else:
                if location_found:
                    if len(locations) == 1:
                        userdata.location = locations[0]
                        return "succeeded"
                    else:
                        userdata.question = 
                        return "failed"
                else:
                    if object_found:
                        ""
                    else:
                        if direction_found:
                        else:
                            if demonstrative_pronoun_found:
                                if demonstrative_pronoun == :
                                    if "but" in trick_words:
                                        idx = locations.index("but")
                                        print("Found at index:", idx)
                                        userdata.location = demonstrative_pronoun[idx]
                                        return "succeeded"
                                    # if "instead"
                                    # elif "not":
                                    # elif "rather":
                                    # elif "morethen":
                                    
                                    # trick_words_dict = {
                                    #     "not": "",
                                    #     "but": ""
                                    #     "rather":"",
                                    #     "morethen":"",
                                    # }    



                


                return "succeeded"
            else:
                print("Not found")
        return "failed"
            # trick_words += word
        
        # for word in words:
        #     value = location.get(word)

        #     if value is not None:
        #         print("Found:", value)
        #         return "succeeded"
        #     else:
        #         print("Not found")
        #         return "failed"

            # userdata.question = ""


location_dict = {
                "dishwasher": "dishwasher",
                "automatic dishwasher": "dishwasher",
                "sink":"sink",
                "table":"table",
                "fridge":"fridge",
                "shelf":"shelf",
            }

trick_words_dict = {
                "not": "",
                "but": ""
                "rather":"",
                "morethen":"",
            }

object_dict = {
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


direction_dict = {
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

demonstrative_pronoun_dict = {
    "here":"",        
    "there":"",      
    "nearby":"",     
    "far":"",        
    "away":"",        
    "along":"",    
    "around":"",     
    "near":"",      
    "next to":"",    
    "beside":"",      
    "beyond":"",      
    "within":"",     
    "without":"",    
    "ahead":"",      
    "behind":"",    
    "off":"",         
    "on":"",        
    "in":"",          
    "out":"",         
    "upstairs":"",    
    "downstairs":"",  
    "indoors":"",    
    "outdoors":"",     
}




            
        

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

# # #& multiple

# #1. nothing (put it, place it)
# f"Need more information. please tell the location like dishwasher."
# # f"dishwasher? sink? fridge? shelf? table?"

# #1. only direction (place behind, behind plz)
# f"{direction} of what or where? Could you explain more?"

# #1. only object (place brush, put cup)
# f"Where shell I place ?"

# #1. only demonstrative_pronoun (place there, place here)
# f"{demonstrative_pronoun}? Please explain more."

# #1. only location ()
# f"I'll put at {location}. Am I right?" #shell I not check?

# #2. direction and object (on the dish, put dish up)
# f"Dish on where?"

# #2. direction and demonstrative_pronoun (up there, down here)
# f"{demonstrative_pronoun} where? Shelf? Table? explain more."

# #2. direction and location (inside of the shelf, shelf inside)
# f"I'll put at {location}. Am I right?"

# #2. direction and Trick words ()

# #2. 

# # Trick words?
# {

# }

# f"while I'm saying you can always interupted. Just say Hi, tiago." #interupped 
# #concurrent state

# {there here }


