import os
import smach
import rospy
from typing import Optional
from sensor_msgs.msg import Image, PointCloud2, CameraInfo


class GetImage(smach.State):
    """
    State for reading an sensor_msgs Image message
    """

    def __init__(self, topic: Optional[str] = None):
        smach.State.__init__(
            self, outcomes=["succeeded", "failed"], output_keys=["img_msg"]
        )

        if topic is None:
            self.topic = (
                "/xtion/rgb/image_raw"
                if "tiago" in os.environ["ROS_MASTER_URI"]
                else "/usb_cam/image_raw"
            )
        else:
            self.topic = topic

    def execute(self, userdata):
        try:
            userdata.img_msg = rospy.wait_for_message(self.topic, Image)
        except Exception as e:
            print(e)
            return "failed"

        return "succeeded"


class GetPointCloud(smach.State):
    """
    State for acquiring a PointCloud2 message.
    """

    def __init__(self, topic: Optional[str] = None):
        smach.State.__init__(
            self, outcomes=["succeeded", "failed"], output_keys=["pcl_msg"]
        )

        if topic is None:
            self.topic = "/xtion/depth_registered/points"
        else:
            self.topic = topic

    def execute(self, userdata):
        try:
            userdata.pcl_msg = rospy.wait_for_message(self.topic, PointCloud2)
        except Exception as e:
            print(e)
            return "failed"

        return "succeeded"


class GetImageAndPointCloud(smach.State):
    """
    State for acquiring Image and PointCloud2 messages simultaneously.
    """

    def __init__(self):
        smach.State.__init__(
            self, outcomes=["succeeded", "failed"], output_keys=["img_msg", "pcl_msg"]
        )

        self.topic1 = "/xtion/rgb/image_raw"
        self.topic2 = "/xtion/depth_registered/points"

    def execute(self, userdata):
        try:
            userdata.img_msg = rospy.wait_for_message(self.topic1, Image)
            userdata.pcl_msg = rospy.wait_for_message(self.topic2, PointCloud2)
        except Exception as e:
            print(e)
            return "failed"

        return "succeeded"


class GetImageAndDepthImage(smach.State):
    """
    State for reading RGB image, depth image and camera info for 3D pose detection
    """

    def __init__(self, camera: str = "xtion"):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            output_keys=["img_msg", "depth_msg", "camera_info"],
        )

        self.camera = camera
        self.image_topic = f"/{camera}/rgb/image_raw"
        self.depth_topic = f"/{camera}/depth_registered/image_raw"
        self.depth_camera_info_topic = f"/{camera}/depth_registered/camera_info"

    def execute(self, userdata):
        try:
            # Get RGB image
            rospy.loginfo(f"Waiting for RGB image from {self.image_topic}")
            userdata.img_msg = rospy.wait_for_message(
                self.image_topic, Image, timeout=5.0
            )

            # Get depth image
            rospy.loginfo(f"Waiting for depth image from {self.depth_topic}")
            userdata.depth_msg = rospy.wait_for_message(
                self.depth_topic, Image, timeout=5.0
            )

            # Get camera info
            rospy.loginfo(
                f"Waiting for camera info from {self.depth_camera_info_topic}"
            )
            userdata.camera_info = rospy.wait_for_message(
                self.depth_camera_info_topic, CameraInfo, timeout=5.0
            )

            rospy.loginfo("Successfully acquired all camera data")
            return "succeeded"

        except Exception as e:
            rospy.logerr(f"Failed to get camera data: {e}")
            return "failed"
