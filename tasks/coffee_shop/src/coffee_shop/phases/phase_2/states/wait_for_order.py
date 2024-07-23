import smach
import json
import rospy
from play_motion_msgs.msg import PlayMotionGoal
import difflib
from std_msgs.msg import Empty, String
import numpy as np
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseGoal
from scipy.spatial.transform import Rotation as R


class WaitForOrder(smach.State):
    def __init__(self, context):
        smach.StateMachine.__init__(self, outcomes=["done"])
        self.context = context
        self.tablet_pub = rospy.Publisher("/tablet/screen", String, queue_size=10)

    def listen(self):
        resp = self.context.speech(True)
        if not resp.success:
            self.context.voice_controller.sync_tts(
                self.context.get_random_retry_utterance()
            )
            return self.listen()
        resp = json.loads(resp.json_response)
        rospy.loginfo(resp)
        return resp

    def affirm(self):
        resp = self.listen()
        if resp["intent"]["name"] not in ["affirm", "deny"]:
            self.context.voice_controller.sync_tts(
                self.context.get_random_retry_utterance()
            )
            return self.affirm()
        return resp["intent"]["name"] == "affirm"

    def execute(self, userdata):
        if self.context.tablet:
            self.context.voice_controller.sync_tts(
                "Please press 'done' when you are ready for me to check the order."
            )
            if self.context.tablet_on_head:
                pm_goal = PlayMotionGoal(motion_name="tablet", skip_planning=True)
                self.context.play_motion_client.send_goal_and_wait(pm_goal)
            else:
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
                ) * R.from_euler("z", 180.0, degrees=True)
                move_base_goal = MoveBaseGoal()
                move_base_goal.target_pose.header.frame_id = "map"
                move_base_goal.target_pose.pose = Pose(
                    position=robot_pose.position,
                    orientation=Quaternion(*target_orientation.as_quat()),
                )
                self.context.move_base_client.send_goal_and_wait(move_base_goal)
                pm_goal = PlayMotionGoal(
                    motion_name="tablet_no_head", skip_planning=True
                )
                self.context.play_motion_client.send_goal_and_wait(pm_goal)

            self.tablet_pub.publish(String("done"))
            rospy.wait_for_message("/tablet/done", Empty)
            return "done"
        else:
            pm_goal = PlayMotionGoal(motion_name="back_to_default", skip_planning=True)
            self.context.play_motion_client.send_goal_and_wait(pm_goal)
            while True:
                rospy.sleep(rospy.Duration(5.0))
                self.context.voice_controller.sync_tts(
                    "Is the order ready to be checked? Please answer with yes or no after the beep."
                )
                if self.affirm():
                    return "done"
