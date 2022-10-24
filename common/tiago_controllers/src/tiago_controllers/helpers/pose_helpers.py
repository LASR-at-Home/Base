import rospy
from geometry_msgs.msg import Pose, Point, PoseWithCovarianceStamped, Quaternion

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


def get_current_pose():
    msg = rospy.wait_for_message("/amcl_pose", PoseWithCovarianceStamped)
    x = round(msg.pose.pose.position.x, 2)
    y = round(msg.pose.pose.position.y, 2)
    quat = msg.pose.pose.orientation
    return x, y, quat