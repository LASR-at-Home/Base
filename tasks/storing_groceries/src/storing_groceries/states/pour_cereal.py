import rospy

import smach
import smach_ros
from shapely import Polygon as ShapelyPolygon

from storing_groceries.states import (
    SelectObject,
)

from lasr_skills import (
    DetectAllInPolygonSensorData,
    PlayMotion,
)

from lasr_manipulation_msgs.srv import (
    Registration,
    RegistrationRequest,
    AddCollisionObject,
    AddCollisionObjectRequest,
)

from lasr_manipulation_msgs.msg import PickAction
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import tf
import moveit_commander


class PourCereal(smach.StateMachine):
    """
    State machine for pouring cereal into a bowl.
    1. detect the container and cereal
    2. grasp the cereal box
    3. move the cereal box to the container location by adding a offset of height
    4. pouring the cereal by flipping the orientation of the pose
    P.S. it might need to change the name of the container to the true label after yolo detection and the offset of the cereal box to the container height.


    """

    def __init__(self) -> None:
        super().__init__(outcomes=["succeeded", "failed"])

        with self:
            smach.StateMachine.add(
                "DETECT_CEREAL_CONTAINER",
                DetectAllInPolygonSensorData(
                    ShapelyPolygon(rospy.get_param("/storing_groceries/table/polygon")),
                    object_filter=["cereal", "container"],
                    min_coverage=0.9,
                    min_confidence=0.1,
                    z_axis=0.8,
                    model="lasr.pt",
                ),
                transitions={"succeeded": "CHECK_DETECTED", "failed": "failed"},
                remapping={"detected_objects": "detected_objects"},
            )

            smach.StateMachine.add(
                "CHECK_DETECTED",
                CheckDetectionResult(),
                transitions={
                    "found_both": "SELECT_CEREAL",
                    "retry": "DETECT_CEREAL_CONTAINER",
                },
                remapping={
                    "detected_objects": "detected_objects",
                    "container_objects": "container_objects",
                },
            )

            smach.StateMachine.add(
                "SELECT_CEREAL",
                SelectObject(target_name="cereal"),
                transitions={"succeeded": "REGISTER_OBJECT", "failed": "failed"},
                remapping={
                    "detected_objects": "detected_objects",
                    "selected_object": "selected_object",
                },
            )

            smach.StateMachine.add(
                "REGISTER_OBJECT",
                smach_ros.ServiceState(
                    "/lasr_manipulation/registration",
                    Registration,
                    request_cb=self._register_object_cb,
                    output_keys=["transform", "scale", "success"],
                    response_slots=["transform", "scale", "success"],
                    input_keys=["masked_cloud", "selected_object"],
                ),
                transitions={
                    "succeeded": "ADD_COLLISION_OBJECT",
                    "preempted": "failed",
                    "aborted": "failed",
                },
            )

            smach.StateMachine.add(
                "ADD_COLLISION_OBJECT",
                smach_ros.ServiceState(
                    "/lasr_manipulation_planning_scene/add_collision_object",
                    AddCollisionObject,
                    request_cb=self._add_collision_object_cb,
                    output_keys=["success"],
                    response_slots=["success"],
                    input_keys=["selected_object", "transform", "scale"],
                ),
                transitions={
                    "succeeded": "PICK_CEREAL",
                    "preempted": "failed",
                    "aborted": "failed",
                },
            )

            smach.StateMachine.add(
                "PICK_CEREAL",
                smach_ros.SimpleActionState(
                    "/lasr_manipulation/pick",
                    PickAction,
                    goal_cb=self._pick_cb,
                    output_keys=["success", "grasp_pose"],
                    result_slots=["success", "grasp_pose"],
                    input_keys=["selected_object", "transform", "scale"],
                ),
                transitions={
                    "succeeded": "MOVE_ABOVE_CONTAINER",
                    "preempted": "SAY_WAIT_CEREAL_GRASP",
                    "aborted": "SAY_WAIT_CEREAL_GRASP",
                },
            )

            smach.StateMachine.add(
                "SAY_WAIT_CEREAL_GRASP",
                Say(text="Carful, I will move my arm."),
                transitions={
                    transitions={
                    "succeeded": f"READY_GRASP_CEREAL",
                    "aborted": f"READY_GRASP_CEREAL",
                    "preempted": f"READY_GRASP_CEREAL",
                    },
                },
            )

            smach.StateMachine.add(
                "READY_GRASP_CEREAL",
                ReadyGraspCereal(),
                transitions={
                    transitions={
                    "succeeded": f"SAY_DONE_READY_GRASP_CEREAL",
                    "failed": f"SAY_DONE_READY_GRASP_CEREAL",
                    },
                },
            )

            smach.StateMachine.add(
                "SAY_DONE_READY_GRASP_CEREAL",
                Say(text="Done. Please put the cereal to my gripper and wait until I close. 5 4 3 2 1"),
                transitions={
                    transitions={
                    "succeeded": f"CLOSE_GRIPER_CEREAL",
                    "aborted": f"CLOSE_GRIPER_CEREAL",
                    "preempted": f"CLOSE_GRIPER_CEREAL",
                    },
                },
            )

            smach.StateMachine.add(
                "CLOSE_GRIPER_CEREAL",
                CloseGriperCereal(),
                transitions={
                    transitions={
                    "succeeded": f"MOVE_ABOVE_CONTAINER",
                    "failed": f"MOVE_ABOVE_CONTAINER",
                    },
                },
            )

            smach.StateMachine.add(
                "MOVE_ABOVE_CONTAINER",
                MoveToAboveContainer(
                    container_key="container_objects",
                    grasp_key="grasp_pose",
                    offset_z=0.15,
                ),
                transitions={
                    "succeeded": "POUR_CEREAL",
                    "failed": "failed",
                },
            )

            smach.StateMachine.add(
                "POUR_CEREAL",
                PlayMotion(motion_name="pour_cereal"),
                transitions={
                    "succeeded": "succeeded",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )

    def _register_object_cb(self, userdata, request):
        return RegistrationRequest(
            userdata.masked_cloud,
            userdata.selected_object[0].name,
            rospy.get_param(
                f"/storing_groceries/objects/{userdata.selected_object[0].name}/scale"
            ),
            25,
        )

    def _add_collision_object_cb(self, userdata, request):
        return AddCollisionObjectRequest(
            userdata.selected_object[0].name,
            userdata.selected_object[0].name,
            userdata.transform,
            userdata.scale,
        )


class MoveToAboveContainer(smach.State):
    """
    move the arm to certain pose:
    the input userdata should contain:
    1. container_objects: list of DetectedObject, from DetectAllInPolygonSensorData
    2. grasp_pose: the orientation of the grasp
    """

    def __init__(
        self, container_key="container_objects", grasp_key="grasp_pose", offset_z=0.15
    ):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=[container_key, grasp_key],
        )
        group_name = "arm_torso"
        self.group = moveit_commander.MoveGroupCommander(group_name)
        self.container_key = container_key
        self.grasp_key = grasp_key
        self.offset_z = offset_z

    def execute(self, userdata):
        container_objects = userdata[self.container_key]
        container_pose = None
        for obj in container_objects:
            if obj.name == "container":
                container_pose = obj.pose
                break

        if container_pose is None:
            rospy.logerr("[MoveToAboveContainer] No container object found!")
            return "failed"

        grasp_pose: PoseStamped = userdata[self.grasp_key]

        target_pose = PoseStamped()
        target_pose.header.frame_id = container_pose.header.frame_id
        target_pose.pose.position = Point(
            x=container_pose.pose.position.x,
            y=container_pose.pose.position.y,
            z=container_pose.pose.position.z + self.offset_z,
        )
        target_pose.pose.orientation = grasp_pose.pose.orientation

        rospy.loginfo(f"[MoveToAboveContainer] Moving to: {target_pose.pose.position}")
        rospy.loginfo(f"[MoveToAboveContainer] Orientation from grasp_pose")

        self.group.set_pose_target(target_pose)
        success = self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()

        return "succeeded" if success else "failed"


class PourCereal(smach.State):
    """
    flip the orientation of the cereal box to pour it into the container.

    """

    def __init__(self, grasp_key="grasp_pose"):
        smach.State.__init__(
            self, outcomes=["succeeded", "failed"], input_keys=[grasp_key]
        )
        self.group = moveit_commander.MoveGroupCommander("arm_torso")
        self.grasp_key = grasp_key

    def execute(self, userdata):
        grasp_pose: PoseStamped = userdata[self.grasp_key]
        q_orig = (
            grasp_pose.pose.orientation.x,
            grasp_pose.pose.orientation.y,
            grasp_pose.pose.orientation.z,
            grasp_pose.pose.orientation.w,
        )

        q_rot = tf.transformations.quaternion_from_euler(-1.57, 0, 0)
        q_new = tf.transformations.quaternion_multiply(q_rot, q_orig)

        pour_pose = PoseStamped()
        pour_pose.header = grasp_pose.header
        pour_pose.pose.position = grasp_pose.pose.position
        pour_pose.pose.orientation = Quaternion(*q_new)

        rospy.loginfo("[PourCereal] Pouring with rotated orientation.")
        self.group.set_pose_target(pour_pose)
        success = self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()

        return "succeeded" if success else "failed"


class SelectObject(smach.State):
    """
    Selects the target object by name.
    """

    def __init__(self, target_name="cereal", use_arm=True):
        super().__init__(
            outcomes=["succeeded", "failed"],
            input_keys=["detected_objects"],
            output_keys=["selected_object", "selected_object_name"],
        )
        self._use_arm = use_arm
        self._target_name = target_name

    def execute(self, userdata):
        if not userdata.detected_objects:
            rospy.logwarn("No detected objects available.")
            return "failed"

        for obj, pcl in userdata.detected_objects:
            if obj.name == self._target_name:
                rospy.loginfo(f"Selected target object: {obj.name}")
                userdata.selected_object = (obj, pcl)
                userdata.selected_object_name = obj.name
                return "succeeded"

        rospy.logwarn(f"Target object '{self._target_name}' not found.")
        return "failed"


class CheckDetectionResult(smach.State):
    def __init__(self):
        super().__init__(
            outcomes=["found_both", "retry"],
            input_keys=["detected_objects"],
            output_keys=["container_objects"],
        )

    def execute(self, userdata):
        has_cereal = False
        has_container = False
        container_objs = []

        for obj, _ in userdata.detected_objects:
            if obj.name == "cereal":
                has_cereal = True
            elif obj.name == "container":
                has_container = True
                container_objs.append(obj)

        userdata.container_objects = container_objs

        if has_cereal and has_container:
            rospy.loginfo("[CheckDetectionSuccess] Found both cereal and container.")
            return "found_both"
        else:
            rospy.logwarn(
                "[CheckDetectionSuccess] Missing cereal or container. Retrying..."
            )
            rospy.sleep(1.0)  # Optional: wait briefly before re-detecting
            return "retry"

class ReadyGraspCereal(smach.State):
    """
    Play motion to grasp cereal.

    """
    def __init__(self,):
        smach.State.__init__(
            self, outcomes=["succeeded", "failed"], input_keys=[]
        )
        self.group = moveit_commander.MoveGroupCommander("arm_torso")

    def execute(self, userdata):
        moveit_commander.roscpp_initialize(sys.argv)

        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        rospy.sleep(1.0)

        rospy.loginfo("Waiting for planning scene to update …")

        # # 1) Attach a small cube to the gripper as a collision object
        # gripper_link = "gripper_tool_link"
        # box_id       = "held_obj"
        # box_pose = PoseStamped()
        # box_pose.header.frame_id    = gripper_link
        # box_pose.pose.position.z    = -0.10   # cube center is 10cm below tool link
        # box_pose.pose.orientation.w = 1.0
        # scene.add_box(box_id, box_pose, size=(0.04, 0.04, 0.04))
        # rospy.loginfo("Added box %s to scene at %s", box_id, box_pose)
        # rospy.sleep(0.3)
        # scene.attach_box(gripper_link, box_id,
        #                 touch_links=robot.get_link_names("gripper"))
        # rospy.loginfo("Attached box %s to gripper link %s", box_id, gripper_link)

        play = actionlib.SimpleActionClient("play_motion", PlayMotionAction)
        play.wait_for_server()

        pre = PlayMotionGoal()
        pre.motion_name   = "home"
        pre.skip_planning = False
        play.send_goal_and_wait(pre)

        pre = PlayMotionGoal()
        pre.motion_name   = "home_to_pregrasp"
        pre.skip_planning = False
        play.send_goal_and_wait(pre)
        rospy.loginfo("Moved to pre-place pose, closing gripper …")

        self.group = moveit_commander.MoveGroupCommander("gripper")

        pre = PlayMotionGoal()
        pre.motion_name   = "open_gripper"
        pre.skip_planning = False
        play.send_goal_and_wait(pre)
        rospy.loginfo("Open gripper …")

        moveit_commander.roscpp_shutdown()

        #rosparam get /play_motion/motions

        return "succeeded"


class CloseGriperCereal(smach.State):
    """
    Play motion to grasp cereal.

    """
    def __init__(self,):
        smach.State.__init__(
            self, outcomes=["succeeded", "failed"], input_keys=[]
        )
        self.group = moveit_commander.MoveGroupCommander("arm_torso")

    def execute(self, userdata):
        moveit_commander.roscpp_initialize(sys.argv)

        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        rospy.sleep(5.0) 

        rospy.loginfo("Waiting for planning scene to update …")

        play = actionlib.SimpleActionClient("play_motion", PlayMotionAction)
        play.wait_for_server()

        self.group = moveit_commander.MoveGroupCommander("gripper")

        pre = PlayMotionGoal()
        pre.motion_name   = "close_gripper"
        pre.skip_planning = False
        play.send_goal_and_wait(pre)
        rospy.loginfo("close gripper …")

        moveit_commander.roscpp_shutdown()

        return "succeeded"



