#!/usr/bin/env python3
import rospy
from pal_interaction_msgs.msg import TtsGoal, TtsAction
import actionlib
from actionlib_msgs.msg import GoalStatus


class Voice:
    def __init__(self):
        self._tts_client = actionlib.SimpleActionClient("/tts", TtsAction)
        self._can_tts = self._tts_client.wait_for_server(rospy.Duration(10.0))

    def __tts(self, text):
        if self._can_tts:
            goal = TtsGoal()
            goal.rawtext.text = text
            goal.rawtext.lang_id = "en_GB"
            self._tts_client.send_goal(goal)
        rospy.loginfo(f"\033[32mTIAGO: {text}\033[0m")

    def sync_tts(self, text):
        self.__tts(text)
        return self._tts_client.wait_for_result()

    def async_tts(self, text):
        self.__tts(text)

    def get_tts_status(self):
        return self._tts_client.get_state()

    def is_running(self):
        return (
            self._tts_client.get_state() == GoalStatus.PENDING
            or self._tts_client.get_state() == GoalStatus.ACTIVE
        )
