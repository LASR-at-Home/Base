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
            output_keys=["location", "question"])

            self.trick_words_found = False
            self.location_found = False                        return "succeeded"

            self.object_found = False
            self.direction_found = False
            self.demonstrative_pronoun_found = False
        
    def execute(self, userdata):
        trick_words = []
        location_value = []
        object_value = []
        direction_value = []
        demonstrative_pronoun_value = []

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

            if  self.trick_words_value is not None:
                print("Found:", trick_words_value)
                trick_words.append(trick_words_value)
                self.trick_words_found = True
            if  self.location_value is not None:
                print("Found:", location_value)
                locations.append(location_value)
                self.location_found = True #self.location_found = True
            if self.object_value is not None:
                print("Found:", object_value)
                objects.append(object_value)
                self.object_found = True
            if self.direction_value is not None:
                print("Found:", direction_value)
                detection.append(direction_value)
                self.direction_found = True
            if  self.demonstrative_pronoun_value is not None:
                print("Found:", demonstrative_pronoun_value)
                demonstrative_pronouns.append(demonstrative_pronoun_value)
                self.demonstrative_pronoun_found = True

            if self.trick_words_found:
                if self.location_found:
                    if "but" in trick_words:
                        idx = demonstrative_pronoun.index("but")
                        print("Found at index:", idx)
                        userdata.location = locations[idx]
                        return "succeeded"
                else:
                    if self.object_found:
                        userdata.question = "Where do you want to put it?"
                        return "failed"
                    else:
                        if self.direction_found:
                            userdata.question = "What place of {directions}?"
                        else:
                            if self.sdemonstrative_pronoun_found:
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
                                return "failed"

            else:
                if self.location_found:
                    if len(locations) == 1:
                        userdata.location = locations[0]
                        return "succeeded"
                    else:
                        location_list_str = ", ".join(locations)
                        userdata.question = f"{location_list_str} which one should I go?" 
                        return "failed"
                else:
                    if self.object_found:
                        userdata.question = f"where do you want put it?"
                        return "failed"
                    else:
                        if self.direction_found:
                            if demonstrative_pronoun_found:
                                userdata.question = f"{demonstrative_pronouns} {directions} of where?"
                                return "failed"
                            else:
                                userdata.question = f"Please say clearly and simple."
                                return "failed" 

                            userdata.question = f"{directions} of what or where?"
                            return "failded"
                        else:
                            if self.demonstrative_pronoun_found:
                                userdata.question = f"{demonstrative_pronoun_found} of what?"
                                return "failed"
                            else:
                                userdata.question = f"Please say clearly and simple."
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

def main():
    import rospy
    import smach

    rospy.init_node("communicate_operators_node")

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=["DONE"])
    sm.userdata.transcribed_speech = "put it on the table not the sink"

    with sm:
        smach.StateMachine.add(
            "COMMUNICATE_OPERATORS",
            CommunicateOperators(),
            transitions={"succeeded": "DONE", "failed": "DONE"},
            remapping={
                "transcribed_speech": "transcribed_speech",
                "location": "location",
                "question": "question",
            },
        )

    # Execute SMACH plan
    outcome = sm.execute()

    # Print result
    location = sm.userdata.get("location", None)
    question = sm.userdata.get("question", None)

    rospy.loginfo(f"State machine finished with outcome: {outcome}")
    if location:
        rospy.loginfo(f"Determined location: {location}")
    if question:
        rospy.loginfo(f"Generated question: {question}")


if __name__ == "__main__":
    main()



location_dict = {
                "dishwasher": "dishwasher",
                "automatic dishwasher": "dishwasher",
                "sink":"sink",
                "table":"table",
                "fridge":"fridge",
                "shelf":"shelf",
            }

trick_words_dict = {
                "not": "not",
                "but": "but",
                "rather":"rather",
                "morethen":"morethen",
            }

object_dict = {
    "brush":"brush",
    "cloth":"cloth",
    "polish":"polish",
    "sponge":"sponge",
    "bowl":"bowl",
    "cup":"cup",
    "fork":"fork",
    "knife":"knife",
    "plate":"plate",
    "spoon":"spoon",
    "coffee":"coffee",
    "coke":"coke",
    "fanta":"fanta",
    "kuat":"kuat",
    "milk":"milk",
    "orange juice":"orange juice",
    "broth":"broth",
    "broth box":"broth box",
    "corn flour":"corn flour",
    "ketchup":"ketchup",
    "mayo":"mayo",
    "oats":"oats",
    "tuna":"tuna",
    "apple":"apple",
    "lemon":"lemon",
    "lime":"lime",
    "pear":"pear",
    "tangerine":"tangerine",
    "cheese snack":"cheese snack",
    "chocolate bar":"chocolate bar",
    "cornflakes":"cornflakes",
    "crisps":"crisps",
    "gum balls":"gum balls",
    "peanuts":"peanuts",
    "pringles":"pringles",
    "bag":"bag",
    "dishwasher tab":"dishwasher tab",
    "dishwasher tab_bag":"dishwasher tab_bag",
    "cornflakes container":"cornflakes container",
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

