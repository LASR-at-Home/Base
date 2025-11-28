import sys
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import tf2_ros as tf

from tf2_geometry_msgs import do_transform_point
from lasr_helper_interfaces.srv import TransformPoint
from geometry_msgs.msg import PointStamped


class TfServer(Node):

    _tf_buffer: tf.Buffer
    _tf_listener: tf.TransformListener

    def __init__(self):
        super().__init__('tf_server')
        
        self._tf_buffer = tf.Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = tf.TransformListener(self._tf_buffer, self)

        self.create_service(TransformPoint, '/tf_server/transform_point', self._transform_point)
        self.get_logger().info('TF Server is ready!')

    def _transform_point(self, request, response):
        """Transform a point from one frame to another using the tf2 library.

        Args:
            request: Request containing the source pointstamped and the target frame.
            response: Response containing the transformed pointstamped.

        Returns:
            TransformPointResponse: Response containing the transformed pointstamped.
        """

        try:
            # Transform the point using tf2
            transform = self._tf_buffer.lookup_transform(
                request.target_frame,
                request.input_point_stamped.header.frame_id,
                request.input_point_stamped.header.stamp,
                timeout=Duration(seconds=1.0)
            )
            
            transformed_point = do_transform_point(
                request.input_point_stamped,
                transform
            )
            
            response.transformed_point_stamped = transformed_point
            return response
            
        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ) as e:
            self.get_logger().error(f'TF error: {e}')
            response.transformed_point_stamped = PointStamped()
            return response


def main(args=None):
    rclpy.init(args=args)
    tf_server = TfServer()
    rclpy.spin(tf_server)
    tf_server.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
