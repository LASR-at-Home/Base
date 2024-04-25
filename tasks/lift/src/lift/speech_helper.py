import rospy
import json


def listen(default):
    resp = default.speech()
    print("Resp success: ", resp.success)
    if not resp.success:
        default.voice.speak("Sorry, I didn't get that")
        return listen(default)
    resp = json.loads(resp.json_response)
    rospy.loginfo(resp)
    return resp


def affirm(default):
    resp = listen(default)
    if resp["intent"]["name"] != "affirm":
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


def hear_wait(default):
    resp = listen(default)
    if resp["intent"]["name"] == "negotiate_lift":
        wait = resp["entities"].get("wait_command", [])
        if not wait:
            default.voice.speak("Sorry, did you say wait? I didn't understand.")
            return hear_wait(default)
        else:
            return True
    else:

        return False


def get_people_number(default):
    resp = listen(default)
    if resp["intent"]["name"] != "negotiate_lift":
        default.voice.speak(
            "Sorry, I misheard you, could you say again how many people?"
        )
        return get_people_number(default)
    people = resp["entities"].get("people", [])
    if not people:
        default.voice.speak("Sorry, could you say again how many people?")
        return get_people_number()
    people_number = int(people[0]["value"])
    default.voice.speak("I hear that there are {} people".format(people_number))
    return people_number


def get_floor(default):
    resp = listen(default)
    if resp["intent"]["name"] != "check_floor_number":
        default.voice.speak("Sorry, I misheard you")
        return get_floor(default)
    floor = resp["entities"].get("floor", [])
    if not floor:
        default.voice.speak("Sorry, I can't figure out that we're talking about floors")
        return get_floor()
    floor_number = int(floor[0]["value"])
    default.voice.speak("I heard that we are on floor {}".format(floor_number))
    default.voice.speak(
        "Is this correct? Please answer yes, that is correct or no, that is wrong"
    )

    answer = affirm(default)
    if answer == "yes":
        default.voice.speak("Cool stuff!")
        return floor_number
    else:
        return "failed"
