#!/usr/bin/env python

import rospy
import rosservice
import numpy as np

# CAN'T USE TF PACKAGES IN MELODIC
# import tf2_ros
# import tf2_geometry_msgs
# from pcl_manipulation.srv import PCLObjectDetection

import actionlib
from pick_up_object.srv import TfTransform, TfTransformRequest, DetectObjects
from yolo_object_detection.srv import YoloDetection
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2, Image
from std_srvs.srv import SetBool, SetBoolRequest, Empty


def activate_robot_navigation(flag=True):
    if '/robot_start_navigation' not in rosservice.get_service_list():
        # Assume simulation
        return
    rospy.wait_for_service('/robot_start_navigation', timeout=10)
    try:
        nav = rospy.ServiceProxy('/robot_start_navigation', SetBool)
        nav_resp = nav(SetBoolRequest(flag))
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def detect_yolo(model="coco"):
    """
        Calls detection server (YOLO)

        Return:
            detection {DetectObjects}
    """
    print('waiting for detect_yolo')
    rospy.wait_for_service('/yolo_detection')
    try:
        detect = rospy.ServiceProxy('/yolo_detection', YoloDetection)
        pcl_msg = rospy.wait_for_message('/xtion/depth_registered/points', PointCloud2)
        img = rospy.wait_for_message('/xtion/rgb/image_raw', Image)
        yolo_resp = detect(img, pcl_msg, model, 0.7, 0.3)
        print('detection done!')
        return yolo_resp
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def detect_person(model="coco"):
    rospy.wait_for_service('/yolo_detection')

    try:
        detect_objects = rospy.ServiceProxy('/yolo_detection', YoloDetection)
        # wait for an image message.
        image_msg = rospy.wait_for_message('/xtion/rgb/image_raw', Image)
        # wait for pcl message.
        pcl_msg = rospy.wait_for_message('/xtion/depth_registered/points', PointCloud2)
        # call the service.
        yolo_resp = detect_objects(image_msg, pcl_msg, model, 0.7, 0.3)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        return []
        
    # filter persons
    persons = []
    if not yolo_resp.detected_objects:
        return persons
    else:
        for obj in yolo_resp.detected_objects:
            if obj.name == 'person':
                persons.append(obj)
    
    return persons

def detect_rcnn():
    """
        Calls detection server (Mask-RCNN)

        Return:
            detection {DetectObjects}
    """

    print('waiting for detect_rcnn')
    rospy.wait_for_service('detect_rcnn', timeout=10)
    try:
        detect = rospy.ServiceProxy('detect_rcnn', DetectObjects)
        resp = detect()
        print('detection done!')
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def detect_pcl():
    """
        Calls naive PCL object detection service

        Return:
            segmentations and centroids
    """
    print('waiting for detect_objects')
    rospy.wait_for_service('pcl_manipulation_detection', timeout=10)
    try:
        detect = rospy.ServiceProxy('pcl_manipulation_detection', PCLObjectDetection)
        pcl = rospy.wait_for_message('/xtion/depth_registered/points', PointCloud2, timeout=10)
        resp = detect(pcl)
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


def tf_transform(target_frame, pose_array=None, pointcloud=None, point=None):
    """
    Arguments:
        target_frame {frame_id} -- frame to transform to
        pose_array {PoseArray} -- array of poses
        pointcloud {PointCloud2}
        point {PointStamped}
    
    Returns:
        response {TfTransformResponse} -- target_pose_array {PoseArray}, target_pointcloud {PointCloud2}, target_point {PointStamped}
    """

    assert pose_array is not None or pointcloud is not None or point is not None

    # print('about to transform to ' + target_frame)
    rospy.wait_for_service('tf_transform', timeout=10)
    try:
        tf_transform_srv = rospy.ServiceProxy('tf_transform', TfTransform)
        request = TfTransformRequest()
        if pose_array is not None:
            request.pose_array = pose_array
        if pointcloud is not None:
            request.pointcloud = pointcloud
        if point is not None:
            request.point = point
        request.target_frame.data = target_frame
        response = tf_transform_srv(request)
        # print('transform done!')
        return response
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def to_frame_pose(pose, source_frame='xtion_depth_optical_frame', target_frame='base_footprint'):
    """
    Arguments:
        pose {Pose} -- pose to convert
        source_frame {frame id} -- original coordinate frame
        target_frame {frame id} -- target coordinate frame
    Return:
        pose {Pose} -- target pose
    """
    tfBuffer = tf2_ros.Buffer()
    # remove '/' from source_frame and target frame to avoid tf2.InvalidArgumentException
    source_frame = source_frame.replace('/', '')
    target_frame = target_frame.replace('/', '')
    try:
        transformation = tfBuffer.lookup_transform(target_frame, source_frame,
        rospy.Time(0), rospy.Duration(0.1))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        print(e)
    
    pose = tf2_geometry_msgs.do_transform_pose(PoseStamped(pose=pose), transformation).pose
    return pose


def play_motion_action(action='home'):
    """
    Arguments:
        action {String} -- predefined actions

    Return:
        bool -- action statusf 
    """


    pm_client = actionlib.SimpleActionClient('/play_motion', PlayMotionAction)
    pm_client.wait_for_server()
    goal = PlayMotionGoal()
    goal.motion_name = action
    goal.skip_planning = False
    goal.priority = 0  # Optional

    print("Sending goal with motion: " + action)
    pm_client.send_goal(goal)

    print("Waiting for result...")
    action_ok = pm_client.wait_for_result(rospy.Duration(30.0))

    state = pm_client.get_state()
    if action_ok:
        print("Action finished succesfully with state: " + str(actionlib.GoalStatus.to_string(state)))
    else:
        rospy.logwarn("Action failed with state: " + str(actionlib.GoalStatus.to_string(state)))
    
    print(action_ok and state == actionlib.GoalStatus.SUCCEEDED)
    return action_ok and state == actionlib.GoalStatus.SUCCEEDED

def clear_costmap():
    """
        Clears costmap using clear_octomap server
    """

    print('waiting for clear_costmap')
    rospy.wait_for_service('/move_base/clear_costmaps', timeout=10)
    try:
        clear_costmap = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        response = clear_costmap()
        rospy.loginfo('clearing costmap done!')
        return response
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)