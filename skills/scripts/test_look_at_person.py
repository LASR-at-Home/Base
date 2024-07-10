#!/usr/bin/env python3

import rospy
import smach
import smach_ros
from lasr_skills.vision import GetPointCloud
from lasr_skills import LookAtPerson


if __name__ == "__main__":
    rospy.init_node("look_at_person")
    sm = smach.StateMachine(outcomes=["succeeded", "failed"])
    with sm:
        smach.StateMachine.add(
            "GET_POINTCLOUD",
            GetPointCloud("/xtion/depth_registered/points"),
            transitions={"succeeded": "succeeded"},
            remapping={"pcl_msg": "pcl_msg"},
        )

    sis = smach_ros.IntrospectionServer("pointcloud_server", sm, "/LOOK_AT_PERSON")
    sis.start()
    sm.execute()
    pcl = sm.userdata.pcl_msg
    sis.stop()

    sm = LookAtPerson(filter=False)
    sm.userdata.pcl_msg = pcl
    sm.userdata.deepface_detection = []
    outcome = sm.execute()
    print(outcome)
