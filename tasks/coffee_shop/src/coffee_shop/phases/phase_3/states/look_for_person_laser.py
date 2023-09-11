#!/usr/bin/env python3
import smach
import rospy
from sensor_msgs.msg import LaserScan, CameraInfo, Image
from std_msgs.msg import String
from geometry_msgs.msg import Point
import numpy as np
import laser_geometry.laser_geometry as lg
import sensor_msgs.point_cloud2 as pc2
from image_geometry import PinholeCameraModel
from play_motion_msgs.msg import PlayMotionGoal
from coffee_shop.srv import LatestTransformRequest, ApplyTransformRequest
import time

def timeit_rospy(method):
        """Decorator for timing ROS methods"""
        def timed(*args, **kw):
            ts = time.time()
            result = method(*args, **kw)
            te = time.time()
            rospy.loginfo('%r  %2.2f s' % (method.__name__, (te - ts)))
            return result
        return timed

class LookForPersonLaser(smach.State):

    def __init__(self, context):
        smach.State.__init__(self, outcomes=['found', 'not found'])
        self.context = context
        self.camera = PinholeCameraModel()
        self.camera.fromCameraInfo(rospy.wait_for_message("/xtion/rgb/camera_info", CameraInfo))
        self.corners = rospy.get_param("/wait/cuboid")

    @timeit_rospy
    def get_points_and_pixels_from_laser(self, msg):
        """ Converts a LaserScan message to a collection of points in camera frame, with their corresponding pixel values in a flat array. The method pads out the points to add vertical "pillars" to the point cloud.

        Args:
            msg (LaserScan): ROS Laser Scan message from /scan topic.
        Returns:
            List: A list of tuples containing all the filtered and padded points in camera frame.
            List: A list of pixel values corresponding to the points in the first list.
        """
        # First get the laser scan points, and then convert to camera frame
        pcl_msg = lg.LaserProjection().projectLaser(msg)
        pcl_points = [p for p in pc2.read_points(pcl_msg, field_names=("x, y, z"), skip_nans=True)]
        
        tf_req = LatestTransformRequest()
        tf_req.target_frame = "xtion_rgb_optical_frame"
        tf_req.from_frame= "base_laser_link"
        t = self.context.tf_latest(tf_req)

        padded_points = []
        pixels = []
        for point in pcl_points:
            # Pad out the points to add vertical "pillars" to the point cloud
            for z in np.linspace(0., 1., 5):
                padded_points.append(Point(x=point[0], y=point[1], z=z))
    
        apply_req = ApplyTransformRequest()
        apply_req.points = padded_points
        apply_req.transform = t.transform
        res = self.context.tf_apply(apply_req)

        padded_converted_points = []
        for p in res.new_points:
            pt = (p.x, p.y, p.z)
            u,v = self.camera.project3dToPixel(pt)
            # Filter out points that are outside the camera frame
            if u >= 0 and u < 640 and v >= 0 and v < 480:
                pixels.append(u)
                pixels.append(v)
                padded_converted_points.append(pt)

        return padded_converted_points, pixels
    
    @timeit_rospy
    def convert_points_to_map_frame(self, points, from_frame="xtion_rgb_optical_frame"):
        """ Converts a list of points in camera frame to a list of points in map frame.

        Args:
            points (List): A list of tuples containing points in camera frame.
        Returns:
            List: A list of tuples containing points in map frame.
        """
        tf_req = LatestTransformRequest()
        tf_req.target_frame = "map"
        tf_req.from_frame= from_frame
        t = self.context.tf_latest(tf_req)

        apply_req = ApplyTransformRequest()
        apply_req.points = [Point(x=point[0], y=point[1], z=point[2]) for point in points]
        apply_req.transform = t.transform
        res = self.context.tf_apply(apply_req)
        converted_points = [(point.x, point.y, point.z) for point in res.new_points]

        return converted_points

    def execute(self, userdata):
        self.context.stop_head_manager("head_manager")
        pm_goal = PlayMotionGoal(motion_name="back_to_default", skip_planning=True)
        self.context.play_motion_client.send_goal_and_wait(pm_goal)
        
        lsr_scan = rospy.wait_for_message("/scan", LaserScan)
        img_msg = rospy.wait_for_message("/xtion/rgb/image_raw", Image)
        
        points, pixels = self.get_points_and_pixels_from_laser(lsr_scan)
        detections = self.context.yolo(img_msg,self.context.YOLO_person_model, 0.3, 0.3)
    
        for detection in detections.detected_objects:
            if detection.name == "person":
                decision = self.context.shapely.are_points_in_polygon_2d_flatarr(detection.xyseg, pixels)
                idx = [idx for idx, el in enumerate(decision.inside) if el]
                filtered_points = self.convert_points_to_map_frame([points[i] for i in idx])
                if self.corners is not None:
                    waiting_area = self.context.shapely.are_points_in_polygon_2d(self.corners, [[p[0], p[1]] for p in filtered_points])
                    idx = [idx for idx, el in enumerate(waiting_area.inside) if el]
                    points_inside_area = [filtered_points[i] for i in idx]
                    if len(points_inside_area):
                        point = np.mean(points_inside_area, axis=0)
                        self.context.publish_person_pose(*point, "map")
                        self.context.new_customer_pose = point.tolist()

                        return 'found'
                else:
                    #mean of filtered points
                    point = np.mean(filtered_points, axis=0)
                    self.context.publish_person_pose(*point, "map")
                    self.context.new_customer_pose = point.tolist()

                    return 'found'

        self.context.start_head_manager("head_manager", '')

        return 'not found'