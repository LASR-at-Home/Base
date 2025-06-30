#!/usr/bin/env python3
import sys
import math
import rospy
import actionlib
import moveit_commander

from geometry_msgs.msg import PoseStamped, Quaternion, PointStamped
from play_motion_msgs.msg import PlayMotionGoal, PlayMotionAction
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose
import tf2_ros

import sys
rospy.logwarn(sys.executable)

from lasr_grasping.srv import PlaceObject, PlaceObjectResponse
import tf.transformations as tft
from visualization_msgs.msg import Marker

VEL_SCALE   = 0.3
ACC_SCALE   = 0.3
TABLE_THICK = 0.01
TABLE_Z     = 0.41
TORSO_LIFT  = 0.25

arm           = None  
play          = None  
scene         = None  
gripper_link  = "gripper_tool_link"
box_id        = "held_obj"
planning_frame = None
marker_pub  = None

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

# TF buffer shared between threads
tf_buffer = tf2_ros.Buffer()

def publish_arrow(pose_msg, pub, ns="place_target", marker_id=0):
    """
    Publish a green arrow marker at *pose_msg* (PoseStamped).
    """
    m = Marker()
    m.header = pose_msg.header
    m.ns     = ns
    m.id     = marker_id
    m.type   = Marker.ARROW
    m.action = Marker.ADD
    m.pose   = pose_msg.pose
    m.scale.x = 0.20       # shaft length
    m.scale.y = 0.03       # shaft diameter
    m.scale.z = 0.03       # head diameter
    m.color.g = 1.0        # green
    m.color.a = 1.0        # fully opaque
    pub.publish(m)

def place_at_service(x, y, z, q):
    """Plan & execute a placement motion. Return True on total success."""
    global arm, play, scene, box_id, gripper_link

    # Cube is fixed 15 cm below the tool link
    box_offset_z = 0

    # Build goal pose (PoseStamped)
    goal = PoseStamped()
    goal.header.frame_id = planning_frame
    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.position.z = z + box_offset_z
    goal.pose.orientation = q

    # Plan to goal
    arm.set_pose_target(goal, gripper_link)
    ok, plan, _, _ = arm.plan()
    if not ok or len(plan.joint_trajectory.points) == 0:
        rospy.logerr("[place_srv] Planning failed")
        return False
    if not arm.execute(plan, wait=True):
        rospy.logerr("[place_srv] Execution failed")
        return False
    arm.stop(); arm.clear_pose_targets(); rospy.sleep(0.5)

    # Detach cube and open gripper
    scene.remove_attached_object(gripper_link, box_id)
    open_goal = PlayMotionGoal()
    open_goal.motion_name = "open_gripper"
    open_goal.skip_planning = False
    play.send_goal_and_wait(open_goal)
    rospy.sleep(0.5)

    home_goal = PlayMotionGoal()
    home_goal.motion_name   = "home"          
    home_goal.skip_planning = False
    play.send_goal_and_wait(home_goal)
    rospy.sleep(0.5)

    return True

def handle_set_pose(req):
    """Callback for SetGripperPose srv."""
    target = req.target  # PoseStamped
    publish_arrow(target, marker_pub)

    # Transform to planning frame if needed
    if target.header.frame_id != planning_frame:
        try:
            tr = tf_buffer.lookup_transform(planning_frame,
                                            target.header.frame_id,
                                            rospy.Time(0),
                                            rospy.Duration(1.0))
            target = do_transform_pose(target, tr)
        except Exception as e:
            rospy.logerr("[place_srv] TF transform failed: %s", str(e))
            return PlaceObjectResponse(False)
        
    q = target.pose.orientation
    norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
    if abs(norm - 1.0) > 1e-3:   
        rospy.logwarn("[place_srv] quaternion ‖q‖=%.4f, auto-normalising", norm)
        q = Quaternion(q.x / norm, q.y / norm, q.z / norm, q.w / norm)
        target.pose.orientation = q

    success = place_at_service(target.pose.position.x,
                               target.pose.position.y,
                               target.pose.position.z,
                               q=target.pose.orientation)
    return PlaceObjectResponse(success)


def main():
    global arm, play, scene, planning_frame, tf_buffer, marker_pub

    rospy.init_node("tiago_place_service_node")
    marker_pub = rospy.Publisher("/place_target_marker",
                                 Marker, queue_size=1)
    moveit_commander.roscpp_initialize(sys.argv)

    # MoveIt interfaces
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    arm   = moveit_commander.MoveGroupCommander("arm_torso")
    arm.set_end_effector_link(gripper_link)
    arm.set_pose_reference_frame("base_footprint")
    arm.set_planning_time(30.0)
    arm.set_num_planning_attempts(10)
    arm.set_max_velocity_scaling_factor(VEL_SCALE)
    arm.set_max_acceleration_scaling_factor(ACC_SCALE)
    arm.set_goal_tolerance(0.01)
    arm.set_goal_orientation_tolerance(0.1)

    planning_frame = arm.get_planning_frame()

    # Attach cube to gripper as collision object
    box_pose = PoseStamped()
    box_pose.header.frame_id    = gripper_link
    box_pose.pose.position.z    = -0.10  # 10 cm below tool link
    box_pose.pose.orientation.w = 1.0
    scene.add_box(box_id, box_pose, size=(0.04, 0.04, 0.04))
    rospy.sleep(0.3)
    scene.attach_box(gripper_link, box_id,
                     touch_links=robot.get_link_names("gripper"))

    # PlayMotion client – move to pre-place pose once
    play = actionlib.SimpleActionClient("play_motion", PlayMotionAction)
    play.wait_for_server()
    pre = PlayMotionGoal(); pre.motion_name = "deliver_preplace_pose"; pre.skip_planning = False
    play.send_goal_and_wait(pre)
    pre = PlayMotionGoal(); pre.motion_name = "home_to_preplace"; pre.skip_planning = False
    play.send_goal_and_wait(pre)

    # Add table collision box
    table_pose = PoseStamped()
    table_pose.header.frame_id = "base_footprint"
    table_pose.pose.position.x = 0.8
    table_pose.pose.position.z = TABLE_Z
    table_pose.pose.orientation.w = 1.0
    scene.add_box("table", table_pose, size=(1.0, 1.2, TABLE_THICK))
    arm.set_support_surface_name("table")

    # Advertise service
    rospy.Service("set_gripper_pose", PlaceObject, handle_set_pose)
    rospy.loginfo("[place_srv] Service \"set_gripper_pose\" ready.")
    rospy.spin()

if __name__ == "__main__":
    main()
