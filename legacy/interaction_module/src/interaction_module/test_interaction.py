#!/usr/bin/env python3
import rospy

rospy.init_node("test")

from interaction_module.srv import AudioAndTextInteraction

speech = rospy.ServiceProxy("/interaction_module", AudioAndTextInteraction)
# print(speech("ROOM_REQUEST", "ask_location", "Can you show me where the  609 office is"))
print(speech("ROOM_REQUEST", "ask_location", "SOUND:PLAYING:PLEASE"))
