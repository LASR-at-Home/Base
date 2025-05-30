#!/usr/bin/env python3

import rospy
import numpy as np
import open3d as o3d
import sensor_msgs.point_cloud2 as pc2

from geometry_msgs.msg import Point, Pose
from shape_msgs.msg import Mesh, MeshTriangle
from moveit_msgs.msg import CollisionObject
from sensor_msgs.msg import PointCloud2

from lasr_manipulation_collision_object.srv import GenerateCollisionObject, GenerateCollisionObjectRequest, GenerateCollisionObjectResponse

def pointcloud2_to_open3d(pc2_msg: PointCloud2) -> o3d.geometry.PointCloud:
    points = np.array([
        [pt[0], pt[1], pt[2]]
        for pt in pc2.read_points(pc2_msg, field_names=("x", "y", "z"), skip_nans=True)
    ])
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    return pcd

def open3d_mesh_to_ros_mesh(o3d_mesh: o3d.geometry.TriangleMesh) -> Mesh:
    mesh = Mesh()
    vertices = np.asarray(o3d_mesh.vertices)
    triangles = np.asarray(o3d_mesh.triangles)

    mesh.vertices = [Point(x=float(x), y=float(y), z=float(z)) for x, y, z in vertices]
    mesh.triangles = [MeshTriangle(vertex_indices=[int(i) for i in tri]) for tri in triangles]
    return mesh

def pointcloud_to_ros_mesh(pcd: o3d.geometry.PointCloud) -> Mesh:
    pcd.estimate_normals()
    mesh, _ = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=8)
    mesh = mesh.simplify_quadric_decimation(10000)
    return open3d_mesh_to_ros_mesh(mesh)

def handle_completion_to_collision(req: GenerateCollisionObjectRequest) -> GenerateCollisionObjectResponse:
    rospy.loginfo(f"[collision_service] Received point cloud for object '{req.object_id}'")


    pcd = pointcloud2_to_open3d(req.cloud)

    mesh = pointcloud_to_ros_mesh(pcd)


    center = np.mean(np.asarray(pcd.points), axis=0)
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = center.tolist()
    pose.orientation.w = 1.0

   
    obj = CollisionObject()
    obj.id = req.object_id
    obj.header.frame_id = req.frame_id
    obj.meshes = [mesh]
    obj.mesh_poses = [pose]
    obj.operation = CollisionObject.ADD

    return GenerateCollisionObjectResponse(object=obj)

def main():
    rospy.init_node("completion_to_collision_service")
    rospy.Service("completion_to_collision_object", GenerateCollisionObject, handle_completion_to_collision)
    rospy.loginfo("[collision_service] Service 'completion_to_collision_object' is ready.")
    rospy.spin()

if __name__ == "__main__":
    main()
