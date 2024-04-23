import smach_ros
from control_msgs.msg import PointHeadGoal, PointHeadAction
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Header


class LookToPoint(smach_ros.SimpleActionState):
    def __init__(self):
        super(LookToPoint, self).__init__(
            "/head_controller/point_head_action",
            PointHeadAction,
            goal_cb=lambda ud, _: PointHeadGoal(
                pointing_frame="head_2_link",
                pointing_axis=Point(1.0, 0.0, 0.0),
                max_velocity=1.0,
                target=ud.pointstampted,
            ),
            input_keys=["pointstampted"],
        )
