import smach
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
import numpy as np
from play_motion_msgs.msg import PlayMotionGoal
from move_base_msgs.msg import MoveBaseGoal
from scipy.spatial.transform import Rotation as R


class DeliverOrder(smach.State):
    def __init__(self, context):
        smach.StateMachine.__init__(self, outcomes=["done"])
        self.context = context

    def execute(self, userdata):
        self.context.voice_controller.sync_tts("I am going to deliver the order")
        location = rospy.get_param(f"/tables/{self.context.current_table}/location")
        position = location["position"]
        orientation = location["orientation"]
        target_orientation = R.from_quat(
            [orientation["x"], orientation["y"], orientation["z"], orientation["w"]]
        )
        target_orientation *= R.from_euler("z", 180.0, degrees=True)
        move_base_goal = MoveBaseGoal()
        move_base_goal.target_pose.header.frame_id = "map"
        move_base_goal.target_pose.pose = Pose(
            position=Point(**position),
            orientation=Quaternion(*target_orientation.as_quat()),
        )
        self.context.move_base_client.send_goal_and_wait(move_base_goal)

        pm_goal = PlayMotionGoal(motion_name="load_unload", skip_planning=True)
        self.context.play_motion_client.send_goal_and_wait(pm_goal)
        self.context.voice_controller.async_tts(
            "I'll give you some time to unload the order..."
        )
        rospy.sleep(rospy.Duration(30.0))
        pm_goal = PlayMotionGoal(motion_name="back_to_default", skip_planning=True)
        self.context.play_motion_client.send_goal_and_wait(pm_goal)
        return "done"
