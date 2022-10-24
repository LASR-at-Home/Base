#!/usr/bin/env python3
import rospy
from pal_interaction_msgs.msg import TtsGoal, TtsAction
import actionlib
from actionlib_msgs.msg import GoalStatus

# TODO: input is real life or simulation, as it will voice will not play otherwise

class Voice:
    def __init__(self):
        self._tts_client = actionlib.SimpleActionClient('/tts', TtsAction)
        print(rospy.get_published_topics(namespace='/tts'))

    def __tts(self, text: str):
        self._tts_client.wait_for_server()
        goal = TtsGoal()
        goal.rawtext.text = text
        goal.rawtext.lang_id = 'en_GB'
        self._tts_client.send_goal(goal)

    def sync_tts(self, text: str) -> bool:
        self.__tts(text)
        return self._tts_client.wait_for_result()

    def async_tts(self, text: str):
        self.__tts(text)

    def get_tts_status(self) -> int:
        return self._tts_client.get_state()

    def is_running(self) -> bool:
        return self._tts_client.get_state() == GoalStatus.PENDING or \
               self._tts_client.get_state() == GoalStatus.ACTIVE

if __name__ == '__main__':
    rospy.init_node("voice_node", anonymous=True)
    voice = Voice()
    voice.sync_tts("hello world nr 1")
    voice.async_tts("hello world nr 2")
    while voice.is_running():
        print("still talking")
