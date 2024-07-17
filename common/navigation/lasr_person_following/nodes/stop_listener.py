#!/usr/bin/env python3
import rospy

from lasr_speech_recognition_msgs.msg import (  # type: ignore
    TranscribeSpeechAction,
    TranscribeSpeechGoal,
)

from std_msgs.msg import Empty

import actionlib

if __name__ == "__main__":
    rospy.init_node("stop_listener")
    finished_pub = rospy.Publisher("/stop_listener/finished", Empty, queue_size=1)
    transcribe_speech_client = actionlib.SimpleActionClient(
        "transcribe_speech", TranscribeSpeechAction
    )
    transcribe_speech_client.wait_for_server()

    while not rospy.is_shutdown():
        transcribe_speech_client.send_goal_and_wait(TranscribeSpeechGoal())
        result = transcribe_speech_client.get_result()
        if "stop" in result.sequence.lower():
            finished_pub.publish(Empty())
    rospy.spin()
