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

    sweep_points_param = rospy.get_param("/receptionist/sweep_points")
    sweep_points = [Point(**point) for point in sweep_points_param]

    seat_area_param = rospy.get_param("/receptionist/seat_area")

    sofa_area_param = rospy.get_param("/receptionist/sofa_area")

    sofa_point_param = rospy.get_param("/receptionist/sofa_point")

    max_people_on_sofa = rospy.get_param("/receptionist/max_people_on_sofa")

    seat_area = Polygon(seat_area_param)
    assert seat_area.is_valid

    sofa_area = Polygon(sofa_area_param)
    assert sofa_area.is_valid

    sofa_point = Point(**sofa_point_param)

    # exclude the sofa area from the seat area
    # seat_area = seat_area.difference(sofa_area)

    receptionist = Receptionist(
        wait_pose,
        wait_area,
        seat_pose,
        sweep_points,
        seat_area,
        sofa_area,
        sofa_point,
        {
            "name": "charlie",
            "drink": "wine",
            "dataset": "receptionist",
            "detection": False,
        },
        max_people_on_sofa=max_people_on_sofa,
    )

    outcome = receptionist.execute()

    rospy.loginfo(f"Receptionist finished with outcome: {outcome}")
    rospy.spin()
