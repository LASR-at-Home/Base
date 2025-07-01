
#!/usr/bin/env python

import sys
import copy
import rospy
import actionlib
import moveit_commander
from geometry_msgs.msg import PointStamped, PoseStamped
from visualization_msgs.msg import Marker
from play_motion_msgs.msg import PlayMotionGoal, PlayMotionAction
import tf2_ros
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose
from moveit_commander.exception import MoveItCommanderException
from trajectory_msgs.msg import JointTrajectory
import math

USE_CLICKED_POINT = True      
TORSO_LIFT        = 0.25       
TABLE_THICK       = 0.01      
TABLE_Z           = 0.41       
VEL_SCALE         = 0.15     
ACC_SCALE         = 0.15
STAMP_DELAY       = 0.25       

clicked_point = None
marker_pub    = None

def clicked_point_cb(msg: PointStamped):
    """Callback for RViz /clicked_point: store the point and publish a green sphere."""
    global clicked_point, marker_pub
    clicked_point = msg
    m = Marker()
    m.header = msg.header
    m.ns, m.id, m.type, m.action = "clicked_points", 0, Marker.SPHERE, Marker.ADD
    m.pose.position, m.pose.orientation.w = msg.point, 1.0
    m.scale.x = m.scale.y = m.scale.z = 0.10
    m.color.r, m.color.a = 1.0, 0.8
    marker_pub.publish(m)

def wait_marker_connection():
    """Block until at least one subscriber is connected to the marker topic."""
    while marker_pub.get_num_connections() == 0 and not rospy.is_shutdown():
        rospy.sleep(0.05)

def publish_arrow(pose: PoseStamped):
    """Publish a green arrow marker at the given pose."""
    a = Marker()
    a.header = pose.header
    a.ns, a.id, a.type, a.action = "clicked_points", 0, Marker.ARROW, Marker.ADD
    a.pose = pose.pose
    a.scale.x = 0.20
    a.scale.y = a.scale.z = 0.03
    a.color.g, a.color.a = 1.0, 0.9
    marker_pub.publish(a)

def add_time_offset(traj: JointTrajectory, offset: float):
    """Add a time offset to every point in the JointTrajectory."""
    for p in traj.points:
        p.time_from_start += rospy.Duration.from_sec(offset)

def place_at(x, y, z,
             arm: moveit_commander.MoveGroupCommander,
             play: actionlib.SimpleActionClient,
             gripper_link: str,
             scene: moveit_commander.PlanningSceneInterface,
             box_id: str):
    """
    Core placement routine:
    1) Compute the tool-frame goal so that the attached cube's center lands at (x,y,z).
    2) Plan and execute a motion to that goal.
    3) Detach the object and open the gripper.
    Returns True on complete success, False otherwise.
    """
    # compute how far below the gripper_link the cube is attached
    box_offset_z = abs(-0.10)  # same as box_pose.pose.position.z magnitude

    # tool_goal is the pose we want the gripper_link to reach
    tool_goal = PoseStamped()
    tool_goal.header.frame_id = arm.get_planning_frame()
    tool_goal.pose.position.x = x 
    tool_goal.pose.position.y = y 
    tool_goal.pose.position.z = z + box_offset_z
    # tool_goal.pose.position.z = z 
    tool_goal.pose.orientation.w = 1.0

    # 1) Plan and execute move to tool_goal
    arm.set_position_target(
        [tool_goal.pose.position.x,
         tool_goal.pose.position.y,
         tool_goal.pose.position.z],
        gripper_link
    )
    ok, plan, _, _ = arm.plan()
    if not ok:
        rospy.logerr("Planning to placement goal failed")
        return False
    if not arm.execute(plan, wait=True):
        rospy.logerr("Execution to placement goal failed")
        return False
    arm.stop()
    arm.clear_pose_targets()
    rospy.sleep(0.5)

    # 2) Detach cube and open gripper
    scene.remove_attached_object(gripper_link, box_id)
    open_goal = PlayMotionGoal()
    open_goal.motion_name   = "open_gripper"
    open_goal.skip_planning = False
    play.send_goal_and_wait(open_goal)
    rospy.sleep(0.5)

    return True

def main():
    global marker_pub, clicked_point

    rospy.init_node("tiago_place_node")
    moveit_commander.roscpp_initialize(sys.argv)

    # set up TF listener and marker publisher
    tf_buffer   = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    marker_pub  = rospy.Publisher("/click_marker",
                                 Marker, queue_size=1, latch=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    rospy.sleep(1.0)

    rospy.loginfo("Waiting for planning scene to update …")

    # 1) Attach a small cube to the gripper as a collision object
    gripper_link = "gripper_tool_link"
    box_id       = "held_obj"
    box_pose = PoseStamped()
    box_pose.header.frame_id    = gripper_link
    box_pose.pose.position.z    = -0.10   # cube center is 10cm below tool link
    box_pose.pose.orientation.w = 1.0
    scene.add_box(box_id, box_pose, size=(0.04, 0.04, 0.04))
    rospy.loginfo("Added box %s to scene at %s", box_id, box_pose)
    rospy.sleep(0.3)
    scene.attach_box(gripper_link, box_id,
                     touch_links=robot.get_link_names("gripper"))
    rospy.loginfo("Attached box %s to gripper link %s", box_id, gripper_link)
    # 2) Set up play_motion client and do pregrasp + close_gripper
    play = actionlib.SimpleActionClient("play_motion", PlayMotionAction)
    play.wait_for_server()
    rospy.loginfo("Connected to play_motion server")
    pre = PlayMotionGoal()
    pre.motion_name   = "deliver_preplace_pose"
    pre.skip_planning = False
    play.send_goal_and_wait(pre)
    rospy.loginfo("Pregrasp motion done, closing gripper …")

    pre = PlayMotionGoal()
    pre.motion_name   = "home_to_preplace"
    pre.skip_planning = False
    play.send_goal_and_wait(pre)
    rospy.loginfo("Moved to pre-place pose, closing gripper …")


    # 3) Define target place pose (can also subscribe to /clicked_point)
    target = PoseStamped()
    target.header.frame_id = "base_footprint"
    target.pose.position.x = 0.6
    target.pose.position.y = 0.0
    target.pose.position.z = 0.4
    target.pose.orientation.w = 1.0

    wait_marker_connection()
    publish_arrow(target)

    if USE_CLICKED_POINT:
        rospy.Subscriber("/clicked_point", PointStamped, clicked_point_cb)
        rospy.loginfo("Waiting for RViz click …")
        while clicked_point is None and not rospy.is_shutdown():
            rospy.sleep(0.05)
        target.pose.position = clicked_point.point
        target.header        = clicked_point.header
        publish_arrow(target)

    # 4) Initialize MoveIt commander for the arm
    arm = moveit_commander.MoveGroupCommander("arm_torso")
    arm.set_end_effector_link(gripper_link)
    arm.set_pose_reference_frame("base_footprint")
    arm.set_planning_time(30.0)
    arm.set_num_planning_attempts(10)
    arm.set_max_velocity_scaling_factor(VEL_SCALE)
    arm.set_max_acceleration_scaling_factor(ACC_SCALE)
    arm.set_goal_tolerance(0.01)
    arm.set_goal_orientation_tolerance(math.pi)

    # wait for current joint states
    while len(arm.get_current_joint_values()) != len(arm.get_active_joints()) \
          and not rospy.is_shutdown():
        rospy.sleep(0.05)

    # 5) Raise the torso
    arm.set_joint_value_target({'torso_lift_joint': TORSO_LIFT})
    arm.go(wait=True)
    arm.stop(); arm.clear_pose_targets()

    # 6) Add the table to the planning scene
    # table_pose = PoseStamped()
    # table_pose.header.frame_id = "base_footprint"
    # table_pose.pose.position.x = 0.8
    # table_pose.pose.position.z = TABLE_Z
    # table_pose.pose.orientation.w = 1.0
    # scene.add_box("table", table_pose, size=(1.0, 1.2, TABLE_THICK))
    # arm.set_support_surface_name("table")


    # transform target into the arm's planning frame if needed
    planning_frame = arm.get_planning_frame()
    if target.header.frame_id != planning_frame:
        tr = tf_buffer.lookup_transform(planning_frame,
                                        target.header.frame_id,
                                        rospy.Time(0),
                                        rospy.Duration(1.0))
        target = do_transform_pose(target, tr)

    # 7) Call the placement function
    success = place_at(
        target.pose.position.x,
        target.pose.position.y,
        target.pose.position.z,
        arm=arm,
        play=play,
        gripper_link=gripper_link,
        scene=scene,
        box_id=box_id
    )
    rospy.loginfo("Place operation %s", "SUCCEEDED" if success else "FAILED")

    # 8) Return to home
    home = PlayMotionGoal()
    home.motion_name   = "home"
    home.skip_planning = False
    play.send_goal_and_wait(home)

    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    main()

