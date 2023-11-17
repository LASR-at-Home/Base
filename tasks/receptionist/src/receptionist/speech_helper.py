import rospy
import json


def listen(default):
    print("trying to listen!")
    resp = default.speech(True)
    print("Resp success: ", resp.success)
    if not resp.success:
        default.voice.speak("Sorry, I didn't get that")
        return listen(default)
    resp = json.loads(resp.json_response)
    rospy.loginfo(resp)
    return resp

def affirm(default):
    resp = listen(default)
    if resp['intent']['name'] != 'affirm':
        default.voice.speak("Sorry, I didn't get that, please say yes or no")
        return affirm(default)
    choices = resp["entities"].get("choice", None)
    if choices is None:
        default.voice.speak("Sorry, I didn't get that")
        return affirm(default)
    choice = choices[0]["value"].lower()
    if choice not in ["yes", "no"]:
        default.voice.speak("Sorry, I didn't get that")
        return affirm(default)
    return choice

def get_drink(default):
    resp = listen(default)
    # if resp['intent']['name'] != 'fav_drink':
    #     return "unknown"
    drink = resp["entities"].get("drink",[])
    if drink is None: 
        return "unknown"
    drink = drink[0]["value"].lower()
    return str(drink)

def get_name(default):
    resp = listen(default)
    # if resp['intent']['name'] != 'name':
    #     return "unknown"
    name = resp["entities"].get("name",[])
    if name is None: 
        return "unknown"
    name = name[0]["value"].lower()
    return str(name)