import rospy

from lang_sam import LangSAM

from lasr_vision_msgs.srv import LangSam, LangSamRequest, LangSamResponse
from sensor_msgs.msg import Image

camera_image = None


def image_callback(image_msg):
    global camera_image
    camera_image = image_msg


def main():
    rospy.Subscriber(f"/xtion/rgb/image_raw", Image, image_callback, queue_size=1)
    client = rospy.ServiceProxy("/lasr_vision/lang_sam", LangSam)

    while True:
        if camera_image is not None:
            req = LangSamRequest(
                image_raw=camera_image,
                prompt="bag",
            )
            resp = client(req)
            print(resp.detections)
    rospy.spin()


if __name__ == "__main__":
    rospy.init_node("lang_sam_example")
    rospy.loginfo("Starting LangSAM example node")
    rospy.loginfo("Waiting for /lasr_vision/lang_sam service...")
    rospy.wait_for_service("/lasr_vision/lang_sam")
    rospy.loginfo("/lasr_vision/lang_sam service is available")
    rospy.loginfo("Starting LangSAM example...")
    main()
