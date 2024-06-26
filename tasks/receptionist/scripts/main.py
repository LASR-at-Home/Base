#!/usr/bin/env python3
import rospy
from receptionist.state_machine import Receptionist
import smach
import smach_ros

from geometry_msgs.msg import Pose, Point, Quaternion
from shapely.geometry import Polygon

if __name__ == "__main__":
    rospy.init_node("receptionist_robocup")
    wait_pose_param = rospy.get_param("/receptionist/wait_pose")

    wait_pose = Pose(
        position=Point(**wait_pose_param["position"]),
        orientation=Quaternion(**wait_pose_param["orientation"]),
    )

    wait_area_param = rospy.get_param("/receptionist/wait_area")
    wait_area = Polygon(wait_area_param)

    seat_pose_param = rospy.get_param("/receptionist/seat_pose")
    seat_pose = Pose(
        position=Point(**seat_pose_param["position"]),
        orientation=Quaternion(**seat_pose_param["orientation"]),
    )

    seat_area_param = rospy.get_param("/receptionist/seat_area")

    seat_area = Polygon(seat_area_param)

    receptionist = Receptionist(
        wait_pose,
        wait_area,
        seat_pose,
        seat_area,
        {
            "name": "charlie",
            "drink": "wine",
            "dataset": "receptionist",
            "confidence": 0.5,
            "attributes": {
                "has_hair": 0.5,
                "hair_shape": "straight hair",
                "hair_colour": "black hair",
                "facial_hair": 0,
                "earrings": 0,
                "necklace": 0,
                "necktie": 0,
                # "height": "unknown",
                "glasses": -0.5,
                "hat": -0.5,
                "dress": 0,
                "top": 0.5,
                "outwear": 0,
                "max_dress": "unknown",
                "max_top": "short sleeved top",
                "max_outwear": "unknown",

                "detection" : True,
            },
        },
    )

    # sis = smach_ros.IntrospectionServer("smach_server", receptionist, "/SM_ROOT")
    # sis.start()
    outcome = receptionist.execute()

    # sis.stop()
    rospy.loginfo(f"Receptionist finished with outcome: {outcome}")
    rospy.spin()
