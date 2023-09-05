import rospy
from geometry_msgs.msg import Pose, Point, Quaternion


def get_pose_from_param(name):
    if not rospy.has_param(name):
        return None
    pose = rospy.get_param(name)
    print(pose)
    return Pose(Point(pose['position']['x'],
                      pose['position']['y'],
                      pose['position']['z']),
                Quaternion(pose['orientation']['x'],
                           pose['orientation']['y'],
                           pose['orientation']['z'],
                           pose['orientation']['w']))

def get_pose_from_param_new(name, col='location'):
    if not rospy.has_param(name):
        return None
    pose = rospy.get_param(name)
    position, orientation = pose[col]["position"], pose[col]["orientation"]
    return Pose(position=Point(**position), orientation=Quaternion(**orientation))


