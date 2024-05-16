import smach
import numpy as np
from play_motion_msgs.msg import PlayMotionGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseGoal
import rospy
from scipy.spatial.transform import Rotation as R


class LoadOrder(smach.State):
    def __init__(self, context):
        smach.StateMachine.__init__(self, outcomes=["done"])
        self.context = context

    def execute(self, userdata):
        self.context.voice_controller.sync_tts(
            "That's the right order, I'll turn around so that you can load it"
        )
        robot_pose = rospy.wait_for_message(
            "/amcl_pose", PoseWithCovarianceStamped
        ).pose.pose
        target_orientation = R.from_quat(
            [
                robot_pose.orientation.x,
                robot_pose.orientation.y,
                robot_pose.orientation.z,
                robot_pose.orientation.w,
            ]
        ) * R.from_euler("z", np.pi, degrees=False)
        move_base_goal = MoveBaseGoal()
        move_base_goal.target_pose.header.frame_id = "map"
        move_base_goal.target_pose.pose = Pose(
            position=robot_pose.position,
            orientation=Quaternion(*target_orientation.as_quat()),
        )
        self.context.move_base_client.send_goal_and_wait(move_base_goal)

        pm_goal = PlayMotionGoal(motion_name="load_unload", skip_planning=True)
        self.context.play_motion_client.send_goal_and_wait(pm_goal)
        self.context.voice_controller.sync_tts(
            "I'll give you some time to load the order..."
        )
        rospy.sleep(rospy.Duration(10.0))
        self.context.start_head_manager("head_manager", "")
        return "done"
