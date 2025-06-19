import rospy
import message_filters

from sensor_msgs.msg import Image, CameraInfo

from lasr_vision_msgs.srv import Recognise3D, Recognise3DRequest


def relay_3d(image_topic: str, depth_topic: str, depth_camera_info_topic: str) -> None:

    recognise = rospy.ServiceProxy("/lasr_vision_reid/recognise", Recognise3D)
    recognise.wait_for_service()
    rospy.loginfo("Service is ready!")

    def detect_cb(image: Image, depth_image: Image, depth_camera_info: CameraInfo):
        request = Recognise3DRequest(
            image_raw=image,
            depth_image=depth_image,
            depth_camera_info=depth_camera_info,
            threshold=0.5,
            target_frame="map",
        )
        response = recognise(request)
        rospy.loginfo(response)

    image_sub = message_filters.Subscriber(image_topic, Image)
    depth_sub = message_filters.Subscriber(depth_topic, Image)
    depth_camera_info_sub = message_filters.Subscriber(
        depth_camera_info_topic, CameraInfo
    )
    ts = message_filters.ApproximateTimeSynchronizer(
        [image_sub, depth_sub, depth_camera_info_sub], 10, 2.0
    )
    ts.registerCallback(detect_cb)


if __name__ == "__main__":
    rospy.init_node("lasr_vision_reid_relay")

    camera = rospy.get_param("~camera", "xtion")
    image_topic = f"/{camera}/rgb/image_raw"
    depth_topic = f"/{camera}/depth_registered/image_raw"
    depth_camera_info_topic = f"/{camera}/depth_registered/camera_info"

    relay_3d(image_topic, depth_topic, depth_camera_info_topic)

    rospy.spin()
