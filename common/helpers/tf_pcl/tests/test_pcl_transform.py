import pytest
import numpy as np
import ros2_numpy as rnp
from geometry_msgs.msg import TransformStamped, Quaternion, Vector3
from sensor_msgs.msg import PointCloud2 
from std_msgs.msg import Header
from scipy.spatial.transform import Rotation as R

from tf_pcl import pcl_transform 
import warnings
warnings.filterwarnings("ignore", category=DeprecationWarning)

def create_test_transform(x, y, z, roll, pitch, yaw, frame_id, child_frame_id):
    """Helper to create a TransformStamped message."""
    t = TransformStamped()
    t.header.stamp.sec = 0
    t.header.stamp.nanosec = 0
    t.header.frame_id = frame_id
    t.child_frame_id = child_frame_id
    
    t.transform.translation = Vector3(x=float(x), y=float(y), z=float(z))
    
    q = R.from_euler('xyz', [roll, pitch, yaw], degrees=False).as_quat()
    t.transform.rotation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    
    return t

def test_pcl_transform_logic():
    """
    Tests if a 2x2 ordered pointcloud is correctly transformed.
    """
    data = np.zeros((2, 2), dtype=[
        ('x', np.float32), 
        ('y', np.float32), 
        ('z', np.float32)
    ])
    
    data[0, 0] = (0.0, 0.0, 0.0)
    data[0, 1] = (1.0, 0.0, 0.0)
    data[1, 0] = (0.0, 1.0, 0.0)
    data[1, 1] = (0.0, 0.0, 1.0)
    
    input_pcl = rnp.msgify(PointCloud2, data)
    input_pcl.header.frame_id = "camera_link"

    transform_msg = create_test_transform(
        x=1.0, y=2.0, z=3.0, 
        roll=0.0, pitch=0.0, yaw=np.pi/2, 
        frame_id="map", child_frame_id="camera_link"
    )

    output_pcl = pcl_transform(input_pcl, transform_msg, "map")

    assert output_pcl.header.frame_id == "map"
    assert output_pcl.height == 2
    assert output_pcl.width == 2
    
    output_arr = rnp.numpify(output_pcl)
    
    np.testing.assert_allclose(
        [output_arr['x'][0,0], output_arr['y'][0,0], output_arr['z'][0,0]],
        [1.0, 2.0, 3.0], atol=1e-5
    )
    np.testing.assert_allclose(
        [output_arr['x'][0,1], output_arr['y'][0,1], output_arr['z'][0,1]],
        [1.0, 3.0, 3.0], atol=1e-5
    )

def test_pcl_preserve_extra_fields():
    """
    Ensures extra fields like intensity are preserved.
    """
    data = np.zeros((2, 2), dtype=[
        ('x', np.float32), 
        ('y', np.float32), 
        ('z', np.float32),
        ('intensity', np.float32)
    ])
    
    data[0,0] = (5.0, 5.0, 5.0, 0.5) 
    
    input_pcl = rnp.msgify(PointCloud2, data)
    input_pcl.header.frame_id = "base"
    
    transform_msg = create_test_transform(0,0,0, 0,0,0, "map", "base")
    
    output_pcl = pcl_transform(input_pcl, transform_msg, "map")
    output_arr = rnp.numpify(output_pcl)
    
    assert 'intensity' in output_arr.dtype.names
    assert output_arr['intensity'][0,0] == 0.5