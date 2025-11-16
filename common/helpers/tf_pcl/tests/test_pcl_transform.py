#!/usr/bin/env python3
import unittest
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import TransformStamped, Quaternion, Vector3
from sensor_msgs_py import point_cloud2
from builtin_interfaces.msg import Time
from std_msgs.msg import Header

from tf_pcl import pcl_transform  


class TestPclTransformSimple(unittest.TestCase):
    
    def create_simple_pointcloud(self):
        """Creates a PointCloud2 with 3D points along x-axis."""
        header = Header()
        header.stamp = Time(sec=0, nanosec=0)
        header.frame_id = "base_link"

        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        # Simple 5 points: [0,0,0], [1,0,0], ...
        points = [(float(i), 0.0, 0.0) for i in range(5)]
        pcl_msg = point_cloud2.create_cloud(header, fields, points)

        return pcl_msg, np.array(points, dtype=np.float32)

    def create_transform(self, tx=1.0, ty=2.0, tz=3.0):
        tf = TransformStamped()
        tf.header.stamp = Time(sec=0, nanosec=0)
        tf.header.frame_id = "base_link"
        tf.child_frame_id = "map"
        tf.transform.translation = Vector3(x=tx, y=ty, z=tz)
        tf.transform.rotation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)  # identity rotation
        return tf

    def test_translation_only(self):
        pcl_in, arr_in = self.create_simple_pointcloud()
        transform = self.create_transform(tx=1.0, ty=2.0, tz=3.0)

        pcl_out = pcl_transform(pcl_in, transform, target_frame="map")

        # Read output points
        out_points = list(point_cloud2.read_points(
            pcl_out, field_names=("x", "y", "z"), skip_nans=True
        ))
        # out_points = list of tuples [(x,y,z), ...]

        expected_points = [
            (i + 1.0, 2.0, 3.0)  # (x+tx, y+ty, z+tz)
            for i in range(5)
        ]

        # Compare length
        self.assertEqual(len(out_points), len(expected_points))

        # Compare each point
        for i, (out_pt, exp_pt) in enumerate(zip(out_points, expected_points)):
            self.assertTrue(
                np.allclose(out_pt, exp_pt, atol=1e-6),
                msg=f"Mismatch at {i}: got {out_pt}, expected {exp_pt}"
            )

        # Verify frame_id
        self.assertEqual(pcl_out.header.frame_id, "map")



if __name__ == '__main__':
    unittest.main()
