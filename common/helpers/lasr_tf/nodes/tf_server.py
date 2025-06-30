import rospy
import tf2_ros as tf

from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point
from lasr_tf.srv import TransformPoint, TransformPointResponse, TransformPointRequest
from geometry_msgs.msg import PointStamped


class TfServer:

    _tf_buffer: tf.Buffer
    _tf_listener: tf.TransformListener

    def __init__(self):

        self._tf_buffer = tf.Buffer(cache_time=rospy.Duration(10.0))
        self._tf_listener = tf.TransformListener(self._tf_buffer)

        rospy.Service(
            "/tf_server/transform_point", TransformPoint, self._transform_point
        )

    def _transform_point(self, req: TransformPointRequest) -> TransformPointResponse:
        """Transform a point from one frame to another using the tf2 library.

        Args:
            req (TransformPointRequest): Request containing the source
            pointstamped and the target frame.

        Returns:
            TransformPointResponse: Response containing the transformed pointstamped.
        """

        try:
            # Transform the point using tf2
            transformed_point = do_transform_point(
                req.input_point_stamped,
                self._tf_buffer.lookup_transform(
                    req.target_frame,
                    req.input_point_stamped.header.frame_id,
                    rospy.Time(1),
                ),
            )
            return TransformPointResponse(transformed_point)
        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ) as e:
            rospy.logerr(f"TF error: {e}")
            return TransformPointResponse(PointStamped())


if __name__ == "__main__":
    rospy.init_node("tf_server")
    tf_server = TfServer()
    rospy.loginfo("TF Server is ready!")
    rospy.spin()
