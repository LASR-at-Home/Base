#!/usr/bin/env python3
import rospy
from receptionist.state_machine import Receptionist
<<<<<<< HEAD
import smach 
=======
import smach
>>>>>>> 5026ebfb0cc02564e84da9d05b79c6aa6d85b8f3
import smach_ros

from geometry_msgs.msg import Pose, Point, Quaternion
from shapely.geometry import Polygon

if __name__ == "__main__":
    rospy.init_node("receptionist_robocup")
<<<<<<< HEAD
   # wait_pose_param = rospy.get_param("/receptionist/wait_pose")
    wait_pose_param = rospy.get_param("/wait_pose/")
=======
    wait_pose_param = rospy.get_param("/receptionist/wait_pose")
    # wait_pose_param = rospy.get_param("/wait_pose/")
>>>>>>> 5026ebfb0cc02564e84da9d05b79c6aa6d85b8f3

    wait_pose = Pose(
        position=Point(**wait_pose_param["position"]),
        orientation=Quaternion(**wait_pose_param["orientation"]),
    )

<<<<<<< HEAD
    wait_area_param = rospy.get_param("/wait_area")
    wait_area = Polygon(wait_area_param)

    seat_pose_param = rospy.get_param("/seat_pose")
=======
    # wait_area_param = rospy.get_param("/wait_area")
    wait_area_param = rospy.get_param("/receptionist/wait_area")
    wait_area = Polygon(wait_area_param)

    # seat_pose_param = rospy.get_param("/seat_pose")
    seat_pose_param = rospy.get_param("/receptionist/seat_pose")
>>>>>>> 5026ebfb0cc02564e84da9d05b79c6aa6d85b8f3
    seat_pose = Pose(
        position=Point(**seat_pose_param["position"]),
        orientation=Quaternion(**seat_pose_param["orientation"]),
    )

<<<<<<< HEAD
    seat_area_param = rospy.get_param("/seat_area")
=======
    # seat_area_param = rospy.get_param("/seat_area")
    seat_area_param = rospy.get_param("/receptionist/seat_area")

>>>>>>> 5026ebfb0cc02564e84da9d05b79c6aa6d85b8f3
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
        },
    )

<<<<<<< HEAD
    sis = smach_ros.IntrospectionServer('smach_server',receptionist,'/SM_ROOT')
=======
    sis = smach_ros.IntrospectionServer("smach_server", receptionist, "/SM_ROOT")
>>>>>>> 5026ebfb0cc02564e84da9d05b79c6aa6d85b8f3
    sis.start()
    outcome = receptionist.execute()

    sis.stop()
    rospy.loginfo(f"Receptionist finished with outcome: {outcome}")
    rospy.spin()
