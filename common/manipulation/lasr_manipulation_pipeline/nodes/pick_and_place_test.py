import rospy
import rospkg
import actionlib

from sensor_msgs.msg import PointCloud2
from play_motion_msgs.msg import PlayMotionGoal, PlayMotionAction
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Empty

import copy

from lasr_manipulation_pipeline import sam

from lasr_manipulation_msgs.srv import (
    Registration,
    DetectAndAddSupportSurface,
    AddCollisionObject,
)

from lasr_manipulation_msgs.msg import PickAction, PickGoal, PlaceAction, PlaceGoal


PACKAGE_PATH: str = rospkg.RosPack().get_path("lasr_manipulation_pipeline")
MESH_NAME: str = "ritz.ply"


class GraspingPipeline:

    def __init__(self):

        self._play_motion_client = actionlib.SimpleActionClient(
            "play_motion", PlayMotionAction
        )
        self._play_motion_client.wait_for_server()
        rospy.loginfo("Setup PlayMotion!")

        self._clear_octomap = rospy.ServiceProxy("/clear_octomap", Empty)
        self._clear_octomap.wait_for_service()

        self._register_object = rospy.ServiceProxy(
            "/lasr_manipulation/registration", Registration
        )
        self._register_object.wait_for_service()
        self._detect_and_add_support_surface = rospy.ServiceProxy(
            "/lasr_manipulation_planning_scene/detect_and_add_support_surface",
            DetectAndAddSupportSurface,
        )
        self._detect_and_add_support_surface.wait_for_service()
        self._add_collision_object = rospy.ServiceProxy(
            "/lasr_manipulation_planning_scene/add_collision_object",
            AddCollisionObject,
        )
        self._add_collision_object.wait_for_service()

        self._clear_planning_scene = rospy.ServiceProxy(
            "/lasr_manipulation_planning_scene/clear", Empty
        )
        self._clear_planning_scene.wait_for_service()

        self._pick_client = actionlib.SimpleActionClient(
            "/lasr_manipulation/pick", PickAction
        )
        self._pick_client.wait_for_server()

        self._place_client = actionlib.SimpleActionClient(
            "/lasr_manipulation/place", PlaceAction
        )
        self._place_client.wait_for_server()

    def _play_motion(self, motion_name: str):
        goal = PlayMotionGoal()
        goal.skip_planning = False
        goal.motion_name = motion_name
        self._play_motion_client.send_goal_and_wait(goal)

    def run(self):
        rospy.loginfo("Sending pregrasp motion...")
        self._play_motion("pregrasp")
        rospy.loginfo("Pregrasp motion finished!")
        rospy.loginfo("Sending open_gripper motion...")
        self._play_motion("open_gripper")
        rospy.loginfo("Pregrasp open_gripper finished!")
        self._clear_octomap()
        self._clear_planning_scene()

        # Get the live sensor point cloud
        pcl = rospy.wait_for_message("/xtion/depth_registered/points", PointCloud2)
        scene_pcl = copy.deepcopy(pcl)

        pcl = sam.segment(pcl)
        # Register mesh with scene and apply the transformation
        response = self._register_object(pcl, MESH_NAME.replace(".ply", ""), False, 25)
        ros_transform = response.transform
        ros_scale = response.scale

        self._detect_and_add_support_surface(
            "table", scene_pcl, MESH_NAME.replace(".ply", ""), ros_transform, ros_scale
        )
        self._add_collision_object(
            MESH_NAME.replace(".ply", ""),
            MESH_NAME.replace(".ply", ""),
            ros_transform,
            ros_scale,
        )

        self._pick_client.send_goal_and_wait(
            PickGoal(
                MESH_NAME.replace(".ply", ""),
                MESH_NAME.replace(".ply", ""),
                ros_transform,
                ros_scale,
            )
        )
        pick_result = self._pick_client.get_result()
        rospy.loginfo(f"Pick result: {pick_result.success}")

        if not pick_result.success:
            rospy.loginfo("Pick failed. Will not place.")
            return

        rospy.loginfo("Sending pregrasp motion...")
        self._play_motion("pregrasp")
        rospy.loginfo("Pregrasp motion finished!")

        self._place_client.send_goal_and_wait(
            PlaceGoal(MESH_NAME.replace(".ply", ""), "table", pick_result.grasp_pose)
        )
        place_result = self._place_client.get_result()
        rospy.loginfo(f"Place result: {place_result.success}")


if __name__ == "__main__":
    rospy.init_node("lasr_grasping_pipeline_test")
    grasping_pipeline = GraspingPipeline()
    grasping_pipeline.run()
