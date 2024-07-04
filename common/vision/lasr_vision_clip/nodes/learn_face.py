import rospy
from lasr_vision_clip import FaceService


if __name__ == "__main__":
    rospy.init_node("clip_vqa_service")
    face_service = FaceService()
    rospy.spin()
