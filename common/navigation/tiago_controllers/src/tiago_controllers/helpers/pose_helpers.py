import rospy
from geometry_msgs.msg import Pose, Point, Quaternion


def get_pose_from_param(name):
    if not rospy.has_param(name):
        return None
    pose = rospy.get_param(name)
    return Pose(Point(pose['position']['x'],
                      pose['position']['y'],
                      pose['position']['z']),
                Quaternion(pose['orientation']['x'],
                           pose['orientation']['y'],
                           pose['orientation']['z'],
                           pose['orientation']['w']))
