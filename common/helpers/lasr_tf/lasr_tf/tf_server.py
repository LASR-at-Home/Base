import rclpy
from rclpy.node import Node

import tf2_ros as tf
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped

from lasr_helper_interfaces.srv import TransformPoint


class TfServer(Node):

    _tf_buffer: tf.Buffer
    _tf_listener: tf.TransformListener

    def __init__(self):
        super().__init__("tf_server")

        self._tf_buffer = tf.Buffer()
        self._tf_listener = tf.TransformListener(self._tf_buffer, self)

        self.create_service(
            TransformPoint, "/tf_server/transform_point", self._transform_point
        )

    def _transform_point(
        self, request: TransformPoint.Request, response: TransformPoint.Response
    ) -> TransformPoint.Response:
        """Transform a point from one frame to another using the tf2 library.

        Args:
            request (TransformPoint.Request): Request containing the source
            pointstamped and the target frame.

        Returns:
            TransformPoint.Response: Response containing the transformed pointstamped.
        """

        try:
            # Transform the point using tf2
            transformed_point = do_transform_point(
                request.input_point_stamped,
                self._tf_buffer.lookup_transform(
                    request.target_frame,
                    request.input_point_stamped.header.frame_id,
                    request.input_point_stamped.header.stamp,
                    rclpy.duration.Duration(seconds=1.0),
                ),
            )
            response.transformed_point_stamped = transformed_point
            return response
        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ) as e:
            self.get_logger().error(f"TF error: {e}")
            response.transformed_point_stamped = PointStamped()
            return response


def main():
    rclpy.init()

    tf_server = TfServer()
    tf_server.get_logger().info("TF Server is ready!", once=True)

    rclpy.spin(tf_server)
    tf_server.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
