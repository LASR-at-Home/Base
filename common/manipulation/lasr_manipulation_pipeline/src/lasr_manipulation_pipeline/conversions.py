import rospy
import tf.transformations
import open3d as o3d
import numpy as np

from geometry_msgs.msg import Pose, Point
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as point_cloud2
from std_msgs.msg import Header
from shape_msgs.msg import Mesh, MeshTriangle


def pose_to_o3d_frame(pose: Pose, size: float = 0.05) -> o3d.geometry.TriangleMesh:
    """
    Convert a ROS Pose to an Open3D coordinate frame mesh.

    Args:
        pose: ROS geometry_msgs.msg.Pose
        size: size of the coordinate frame axes

    Returns:
        o3d.geometry.TriangleMesh coordinate frame mesh transformed to pose
    """
    # Extract translation
    t = np.array([pose.position.x, pose.position.y, pose.position.z])

    # Extract rotation matrix from quaternion
    q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    R = tf.transformations.quaternion_matrix(q)[:3, :3]

    # Create 4x4 transformation matrix
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t

    # Create Open3D coordinate frame and apply transform
    frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=size)
    frame.transform(T)

    return frame


def ros_to_o3d(pcl_msg: PointCloud2) -> o3d.geometry.PointCloud:
    points = list(
        point_cloud2.read_points(pcl_msg, field_names=("x", "y", "z"), skip_nans=True)
    )
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.array(points))
    return pcd


def np_to_ros(points: np.ndarray, frame_id: str) -> PointCloud2:
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id

    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]

    # pack the points into a list of tuples
    points_list = [tuple(p) for p in points]

    pcl2_msg = point_cloud2.create_cloud(header, fields, points_list)

    return pcl2_msg


def o3d_mesh_to_ros_mesh(o3d_mesh: o3d.geometry.TriangleMesh) -> Mesh:
    mesh = Mesh()
    vertices = np.asarray(o3d_mesh.vertices)
    triangles = np.asarray(o3d_mesh.triangles)

    mesh.vertices = [Point(x=float(x), y=float(y), z=float(z)) for x, y, z in vertices]
    mesh.triangles = [
        MeshTriangle(vertex_indices=[int(i) for i in tri]) for tri in triangles
    ]
    return mesh
