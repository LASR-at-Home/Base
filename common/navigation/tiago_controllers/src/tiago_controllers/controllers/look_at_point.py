#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge3 import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from control_msgs.msg import PointHeadAction, PointHeadGoal
from actionlib import SimpleActionClient
from std_msgs.msg import Header
from image_transport import ImageTransport, TransportHints
import sys


# Constants
window_name = "Inside of TIAGo's head"
camera_frame = "/xtion_rgb_optical_frame"
image_topic = "/xtion/rgb/image_raw"
camera_info_topic = "/xtion/rgb/camera_info"

# Intrinsic parameters of the camera
camera_intrinsics = None

# Callback for every new image received
def image_callback(img_msg):
    global camera_intrinsics
    latest_image_stamp = img_msg.header.stamp

    bridge = CvBridge()
    cv_img = bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
    cv2.imshow(window_name, cv_img)
    cv2.waitKey(0)

    camera_intrinsics = cv2.getOptimalNewCameraMatrix(
        camera_intrinsics, camera_intrinsics, (img_msg.width, img_msg.height), 0)

    if camera_intrinsics is not None:
        u, v = cv2.selectROI(cv_img)
        rospy.loginfo("Pixel selected ({}, {}). Making TIAGo look to that direction".format(u, v))

        point_stamped = PointStamped()
        point_stamped.header.frame_id = camera_frame
        point_stamped.header.stamp = latest_image_stamp

        x = (u - camera_intrinsics[0, 2]) / camera_intrinsics[0, 0]
        y = (v - camera_intrinsics[1, 2]) / camera_intrinsics[1, 1]
        Z = 1.0
        point_stamped.point.x = x * Z
        point_stamped.point.y = y * Z
        point_stamped.point.z = Z

        goal = PointHeadGoal()
        goal.pointing_frame = camera_frame
        goal.pointing_axis.x = 0.0
        goal.pointing_axis.y = 0.0
        goal.pointing_axis.z = 1.0
        goal.min_duration = rospy.Duration(10)
        goal.max_velocity = 0.25
        goal.target = point_stamped

        point_head_client.send_goal(goal)
        rospy.sleep(0.5)

# OpenCV callback function for mouse events on a window
def onMouse(event, u, v, flags, param):
    if event != cv2.EVENT_LBUTTONDOWN:
        return

    rospy.loginfo("Pixel selected ({}, {}). Making TIAGo look in that direction".format(u, v))

    point_stamped = PointStamped()
    point_stamped.header.frame_id = camera_frame
    point_stamped.header.stamp = rospy.Time.now()

    # Compute normalized coordinates of the selected pixel
    x = (u - camera_intrinsics[0, 2]) / camera_intrinsics[0, 0]
    y = (v - camera_intrinsics[1, 2]) / camera_intrinsics[1, 1]
    Z = 1.0  # Define an arbitrary distance
    point_stamped.point.x = x * Z
    point_stamped.point.y = y * Z
    point_stamped.point.z = Z

    goal = PointHeadGoal()
    goal.pointing_frame = camera_frame
    goal.pointing_axis.x = 0.0
    goal.pointing_axis.y = 0.0
    goal.pointing_axis.z = 1.0
    goal.min_duration = rospy.Duration(1.0)
    goal.max_velocity = 0.25
    goal.target = point_stamped

    point_head_client.send_goal(goal)
    rospy.sleep(0.5)

# Set mouse handler for the window
cv2.setMouseCallback(window_name, onMouse)

if __name__ == "__main__":
    rospy.init_node('look_to_point')
    rospy.loginfo("Starting look_to_point application ...")
    # Create a ROS action client to move TIAGo's head
    point_head_client = SimpleActionClient("/head_controller/point_head_action", PointHeadAction)
    rospy.loginfo("Creating action client to head controller ...")

    iterations = 0
    max_iterations = 3
    while (not point_head_client.wait_for_server(rospy.Duration(2.0))) and rospy.is_shutdown() and iterations < max_iterations:
        rospy.logdebug("Waiting for the point_head_action server to come up")
        iterations += 1

    if iterations == max_iterations:
        raise RuntimeError("Error in createPointHeadClient: head controller action server not available")

    # Create the window to show TIAGo's camera images
    cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)

    # Set mouse handler for the window
    cv2.setMouseCallback(window_name, onMouse)

    # Define ROS topic from where TIAGo publishes images
    image_transporter = ImageTransport(rospy)
    transport_hint = TransportHints("compressed")

    rospy.loginfo("Subscribing to {} ...".format(image_topic))
    image_subscriber = image_transporter.subscribe(image_topic, Image, image_callback, transport_hint)

    # Enter a loop that processes ROS callbacks. Press CTRL+C to exit the loop
    rospy.spin()

    cv2.destroyWindow(window_name)
