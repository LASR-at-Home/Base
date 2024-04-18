import rospy

from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped

from collections import defaultdict

from typing import Union

publisher_counts = defaultdict(int)


def create_marker(
    point_stamped: PointStamped,
    idx: int,
    r: float = 0.0,
    g: float = 1.0,
    b: float = 0.0,
):
    marker_msg = Marker()
    marker_msg.header.frame_id = point_stamped.header.frame_id
    marker_msg.header.stamp = point_stamped.header.stamp
    marker_msg.id = idx
    marker_msg.type = Marker.SPHERE
    marker_msg.action = Marker.ADD
    marker_msg.pose.position = point_stamped.point
    marker_msg.pose.orientation.w = 1.0
    marker_msg.scale.x = 0.1
    marker_msg.scale.y = 0.1
    marker_msg.scale.z = 0.1
    marker_msg.color.a = 1.0
    marker_msg.color.r = r
    marker_msg.color.g = g
    marker_msg.color.b = b
    return marker_msg


def create_and_publish_marker(
    publisher: rospy.Publisher,
    point_stamped: PointStamped,
    idx: Union[int, None] = None,
    r: float = 0.0,
    g: float = 1.0,
    b: float = 0.0,
):
    if idx is None:
        global publisher_counts
        idx = publisher_counts[publisher]
        publisher_counts[publisher] += 1

    marker_msg = create_marker(point_stamped, idx, r, g, b)
    publisher.publish(marker_msg)
