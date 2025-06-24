#!/usr/bin/env python

# import sys
# import copy
# import rospy
# import actionlib
# import moveit_commander
# from geometry_msgs.msg import PointStamped, PoseStamped
# from visualization_msgs.msg import Marker
# from play_motion_msgs.msg import PlayMotionGoal, PlayMotionAction
# import tf2_ros
# from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose
# from trajectory_msgs.msg import JointTrajectory
# import math
# import random
# import control_msgs.msg 

# from moveit_msgs.msg import Constraints, PositionConstraint
# from shape_msgs.msg import SolidPrimitive
# from geometry_msgs.msg import Pose as GPose

# # ----------------- 全局配置 -----------------
# USE_CLICKED_POINT = True      
# TORSO_LIFT        = 0.35       # 抬高躯干到 0.35 m
# VEL_SCALE         = 0.15     
# ACC_SCALE         = 0.15
# CLICK_OFFSET_Z    = 0.10      # 点击点上的默认偏移
# SAMPLE_TOLERANCE  = 0.20      # 20 cm 采样半径
# SAMPLE_TRIES      = 30        # 采样次数

# clicked_point = None
# marker_pub    = None

# def clicked_point_cb(msg: PointStamped):
#     """Callback for RViz /clicked_point: 记录点击点并显示小球"""
#     global clicked_point, marker_pub
#     clicked_point = msg
#     m = Marker()
#     m.header = msg.header
#     m.ns, m.id, m.type, m.action = "clicked_points", 0, Marker.SPHERE, Marker.ADD
#     m.pose.position, m.pose.orientation.w = msg.point, 1.0
#     m.scale.x = m.scale.y = m.scale.z = 0.10
#     m.color.r, m.color.a = 1.0, 0.8
#     marker_pub.publish(m)
#     rospy.loginfo("Clicked point: x=%.3f, y=%.3f, z=%.3f",
#                   msg.point.x, msg.point.y, msg.point.z)

# def wait_marker_connection():
#     """Block until RViz subscribes to /click_marker"""
#     while marker_pub.get_num_connections() == 0 and not rospy.is_shutdown():
#         rospy.sleep(0.05)

# def publish_arrow(pose: PoseStamped):
#     """在 RViz 发布绿色箭头表示当前目标"""
#     a = Marker()
#     a.header = pose.header
#     a.ns, a.id, a.type, a.action = "clicked_points", 0, Marker.ARROW, Marker.ADD
#     a.pose = pose.pose
#     a.scale.x = 0.20
#     a.scale.y = a.scale.z = 0.03
#     a.color.g, a.color.a = 1.0, 0.9
#     marker_pub.publish(a)

# def find_reachable_plan(group, x, y, z, tol, tries):
#     """
#     在 (x,y,z) ± tol 范围内随机采样 tries 次，
#     每次调用 set_pose_target + plan，返回第一个可用的 plan
#     """
#     for i in range(tries):
#         dx = (random.random()*2 - 1) * tol
#         dy = (random.random()*2 - 1) * tol
#         dz = (random.random()*2 - 1) * tol
#         sample = PoseStamped()
#         sample.header.frame_id = group.get_planning_frame()
#         sample.pose.position.x = x + dx
#         sample.pose.position.y = y + dy
#         sample.pose.position.z = z + dz
#         sample.pose.orientation.w = 1.0
#         rospy.loginfo("  Sample #%d: x=%.3f y=%.3f z=%.3f", i+1,
#                       sample.pose.position.x,
#                       sample.pose.position.y,
#                       sample.pose.position.z)
#         group.set_start_state_to_current_state()
#         group.set_pose_target(sample)
#         ok, plan, _, _ = group.plan()
#         if ok and plan.joint_trajectory.points:
#             return plan
#     return None

# def place_at(x, y, z,
#              arm: moveit_commander.MoveGroupCommander,
#              play: actionlib.SimpleActionClient,
#              gripper_link: str,
#              scene: moveit_commander.PlanningSceneInterface,
#              box_id: str):
#     """
#     尝试在球体容差范围内采样可达目标并执行：
#     1) 随机采样 IK + 规划
#     2) 找到就 execute
#     3) 拆卸并开夹爪
#     """
#     rospy.loginfo("Waiting for controller to settle …")
#     rospy.sleep(1.0)
#     arm.stop(); arm.clear_pose_targets()
#     rospy.sleep(0.2)

#     # 采样寻找可达 plan
#     rospy.loginfo("Sampling reachable pose within ±%.2f m …", SAMPLE_TOLERANCE)
#     plan = find_reachable_plan(arm, x, y, z, SAMPLE_TOLERANCE, SAMPLE_TRIES)
#     if plan is None:
#         rospy.logerr("在 ±%.2f m 范围内没有找到可达路径", SAMPLE_TOLERANCE)
#         return False

#     # 执行采样到的路径
#     rospy.loginfo("Execute sampled plan")
#     if not arm.execute(plan, wait=True):
#         rospy.logerr("执行 plan 失败")
#         return False
#     arm.stop(); arm.clear_pose_targets()
#     rospy.sleep(0.5)

#     # 拆物 & 开夹爪
#     scene.remove_attached_object(gripper_link, box_id)
#     open_goal = PlayMotionGoal()
#     open_goal.motion_name   = "open_gripper"
#     open_goal.skip_planning = False
#     play.send_goal_and_wait(open_goal)
#     rospy.sleep(0.5)
#     return True

# def debug_goal_cb(msg):
#     """监控任何向 controller 下发的 goal"""
#     rospy.logwarn(">>> New controller goal at %.3f s from %s",
#                   rospy.Time.now().to_sec(),
#                   msg._connection_header.get('callerid','?'))

# def main():
#     global marker_pub, clicked_point

#     rospy.init_node("tiago_place_node")
#     # 监听 controller goal，帮助调试
#     rospy.Subscriber("/arm_torso_controller/follow_joint_trajectory/goal",
#                      control_msgs.msg.FollowJointTrajectoryActionGoal,
#                      debug_goal_cb, queue_size=1)

#     moveit_commander.roscpp_initialize(sys.argv)

#     # TF & marker publisher
#     tf_buffer   = tf2_ros.Buffer()
#     tf_listener = tf2_ros.TransformListener(tf_buffer)
#     marker_pub  = rospy.Publisher("/click_marker",
#                                  Marker, queue_size=1, latch=True)

#     robot = moveit_commander.RobotCommander()
#     scene = moveit_commander.PlanningSceneInterface()
#     rospy.sleep(1.0)

#     # 1) Attach cube
#     gripper_link = "gripper_tool_link"
#     box_id       = "held_obj"
#     box_pose = PoseStamped()
#     box_pose.header.frame_id    = gripper_link
#     box_pose.pose.position.z    = -0.10
#     box_pose.pose.orientation.w = 1.0
#     scene.add_box(box_id, box_pose, size=(0.04,0.04,0.04))
#     scene.attach_box(gripper_link, box_id,
#                      touch_links=robot.get_link_names("gripper"))

#     # 2) Pregrasp motions
#     play = actionlib.SimpleActionClient("play_motion", PlayMotionAction)
#     play.wait_for_server()
#     for m in ["deliver_preplace_pose","home_to_preplace"]:
#         goal = PlayMotionGoal(); goal.motion_name=m; goal.skip_planning=False
#         play.send_goal_and_wait(goal)

#     # 3) 点击或预设 target
#     wait_marker_connection()
#     target = PoseStamped(); target.header.frame_id="base_footprint"
#     target.pose.position.x = 0.6; target.pose.position.y = 0.0
#     target.pose.position.z = 0.4; target.pose.orientation.w = 1.0

#     if USE_CLICKED_POINT:
#         rospy.Subscriber("/clicked_point", PointStamped, clicked_point_cb)
#         rospy.loginfo("Waiting for RViz click …")
#         while clicked_point is None and not rospy.is_shutdown():
#             rospy.sleep(0.05)
#         target.pose.position = clicked_point.point
#         target.header        = clicked_point.header
#     rospy.loginfo("Final placement target: x=%.3f y=%.3f z=%.3f",
#                   target.pose.position.x,
#                   target.pose.position.y,
#                   target.pose.position.z)

#     publish_arrow(target)

#     # 4) 初始化 MoveIt
#     arm = moveit_commander.MoveGroupCommander("arm_torso")
#     arm.set_end_effector_link(gripper_link)
#     arm.set_pose_reference_frame("base_footprint")
#     arm.set_planning_time(30.0)
#     arm.set_max_velocity_scaling_factor(VEL_SCALE)
#     arm.set_max_acceleration_scaling_factor(ACC_SCALE)

#     # 抬高躯干到 TORSO_LIFT
#     arm.set_joint_value_target({'torso_lift_joint': TORSO_LIFT})
#     arm.go(wait=True); arm.stop(); arm.clear_pose_targets()
#     rospy.loginfo("Torso lifted to %.2f m", TORSO_LIFT)

#     # 5) 调用 place_at
#     success = place_at(
#         target.pose.position.x,
#         target.pose.position.y,
#         target.pose.position.z + CLICK_OFFSET_Z,
#         arm=arm, play=play,
#         gripper_link=gripper_link,
#         scene=scene, box_id=box_id
#     )
#     rospy.loginfo("Place operation %s", 
#                   "SUCCEEDED" if success else "FAILED")

#     # 6) 返回 home
#     home_goal = PlayMotionGoal()
#     home_goal.motion_name   = "home"
#     home_goal.skip_planning = False
#     play.send_goal_and_wait(home_goal)

#     moveit_commander.roscpp_shutdown()

# if __name__ == "__main__":
#     main()


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

    rospy.logerr("move to the placement goal succeeded, now detaching the object")

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
    rospy.loginfo("Pregrasp motion done …")

    pre = PlayMotionGoal()
    pre.motion_name   = "home_to_preplace"
    pre.skip_planning = False
    play.send_goal_and_wait(pre)
    rospy.loginfo("Moved to pre-place pose …")


    # 3) Define target place pose (can also subscribe to /clicked_point)
    target = PoseStamped()
    target.header.frame_id = "base_footprint"
    target.pose.position.x = 0.6
    target.pose.position.y = 0.0
    target.pose.position.z = 0.4
    target.pose.orientation.w = 1.0

    # wait_marker_connection()
    # publish_arrow(target)

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

    # # wait for current joint states
    # while len(arm.get_current_joint_values()) != len(arm.get_active_joints()) \
    #       and not rospy.is_shutdown():
    #     rospy.sleep(0.05)

    # # 5) Raise the torso
    # arm.set_joint_value_target({'torso_lift_joint': TORSO_LIFT})
    # arm.go(wait=True)
    # arm.stop(); arm.clear_pose_targets()

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
