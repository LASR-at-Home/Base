import rospy
from sensor_msgs.msg import Image
from lasr_vision_msgs.srv import AddFace, AddFaceRequest
import time


def add_face(name: str, num_images: int, image_topic: str):
    rospy.wait_for_service("/lasr_vision_reid/add_face")
    add_face_srv = rospy.ServiceProxy("/lasr_vision_reid/add_face", AddFace)

    images_collected = 0
    last_processed_time = 0.0

    def handle_image(image: Image):
        nonlocal images_collected, last_processed_time
        current_time = time.time()
        if current_time - last_processed_time < 1.0:
            return

        if images_collected >= num_images:
            rospy.loginfo("Collected required number of images. Unsubscribing.")
            image_sub.unregister()
            return

        req = AddFaceRequest()
        req.image_raw = image
        req.name = name

        try:
            resp = add_face_srv(req)
            if resp.success:
                images_collected += 1
                last_processed_time = current_time
                rospy.loginfo(
                    f"Added face '{name}' for image {images_collected}/{num_images}"
                )
            else:
                rospy.logwarn("Failed to add face for this image.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    image_sub = rospy.Subscriber(image_topic, Image, handle_image)
    rospy.spin()


if __name__ == "__main__":
    rospy.init_node("lasr_vision_reid_add_face")
    camera = rospy.get_param("~camera", "xtion")
    name = rospy.get_param("~name", "jared")
    num_images = rospy.get_param("~num_images", 10)
    image_topic = f"/{camera}/rgb/image_raw"

    add_face(name, num_images, image_topic)
