#!/usr/bin/env python3
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image, PointCloud2
from object_interest_tracking.srv import Tdr, TdrResponse
from cv_bridge3 import CvBridge
from lasr_vision_msgs.srv import YoloDetection
from geometry_msgs.msg import Point, PointStamped
import ros_numpy as rnp
from std_msgs.msg import String
from tf_module.srv import TfTransform, TfTransformRequest
from tiago_controllers import BaseController
from math import acos
from lasr_shapely import LasrShapely

from tiago_controllers.helpers.pose_helpers import get_pose_from_param

from lasr_object_detection_yolo.detect_objects_v8 import detect_objects, perform_detection, debug

def estimate_pose(pcl_msg, cv_im, detection):
    tf = rospy.ServiceProxy("/tf_transform", TfTransform)
    contours = np.array(detection).reshape(-1, 2)
    mask = np.zeros((cv_im.shape[0], cv_im.shape[1]), np.uint8)
    cv2.fillPoly(mask, pts=[contours], color=(255, 255, 255))
    indices = np.argwhere(mask)
    if indices.shape[0] == 0:
        return np.array([np.inf, np.inf, np.inf])
    pcl_xyz = rnp.point_cloud2.pointcloud2_to_xyz_array(pcl_msg, remove_nans=False)

    xyz_points = []
    for x, y in indices:
        x, y, z = pcl_xyz[x][y]
        xyz_points.append([x, y, z])

    x, y, z = np.nanmean(xyz_points, axis=0)
    centroid = PointStamped()
    centroid.point = Point(x, y, z)
    centroid.header = pcl_msg.header
    tf_req = TfTransformRequest()
    tf_req.target_frame = String("map")
    tf_req.point = centroid
    tf_req.source_frame = String(pcl_msg.header.frame_id)
    response = tf(tf_req)
    return response.target_point.point


def toNPArray(a):
    return np.array([a.x, a.y])


def orderIndices(current, previous):
    if len(current) == 1 and len(previous) == 1:
        return [0]

    indices = []
    if len(current) == len(previous):
        for ma in range(len(current)):
            diffs = list(map(lambda lo: np.linalg.norm(toNPArray(current[ma]) - toNPArray(lo)), previous))
            indices.append(diffs.index(min(diffs)))

    return indices


def getPoseDiff(lastRecorded, x, y):
    distances = []
    for i in lastRecorded:
        distances.append(np.linalg.norm(toNPArray(i) - np.array([x, y])))
    return distances




def v2(req):
    cvBridge = CvBridge()
    shapley = LasrShapely()
    rospy.wait_for_service('/yolov8/detect')
    corners = rospy.get_param("/corners_arena")
    print(corners, "corners")

    objectRecognitionService = rospy.ServiceProxy('/yolov8/detect', YoloDetection)
    basePose = get_pose_from_param("/phase3_lift/pose")
    print(basePose, "base pose")

    # BaseController().sync_to_pose(basePose)

    headPoint = None
    found = False
    ANGLE_THRESHOLD = 10
    REACHING_THRESHOLDS = 2

    while not found:
        rospy.sleep(0.5)
        locs = []
        for i in range(2):
            img_msg = rospy.wait_for_message('/xtion/rgb/image_raw', Image)
            img_depth_msg = rospy.wait_for_message('/xtion/depth_registered/points', PointCloud2)


            currentLocs = list(filter(lambda i: i.name == "person",
                                      objectRecognitionService(img_msg, 'yolov8n-seg.pt', 0.7, 0.3).detected_objects))

            cm = []
            rospy.logwarn("0")

            for i2 in range(len(currentLocs)):
                # Assume 2 ppl!
                pose = estimate_pose(img_depth_msg, cvBridge.imgmsg_to_cv2_np(img_msg), currentLocs[i2].xyseg)
                if (shapley.is_point_in_polygon_2d(corners, pose.x, pose.y)):
                    cm.append(pose)

            locs.append(cm)
            rospy.logwarn(len(locs[0]))

            print("vecs11")

            if i == 1 and len(locs[0]) > 0 and len(locs[0]) == len(locs[1]):
                # swap if needed
                newIndices = orderIndices(cm, locs[0])

                if (newIndices == [0]):
                    locs.append(cm)
                else:
                    locs.append([cm[i] for i in newIndices])

                # CALC VECS
                vecs = []
                for i in range(len(locs[1])):
                    rospy.logerr(locs[1][i])
                    a2 = toNPArray(locs[1][i]) - toNPArray(locs[0][i])
                    vecs.append(a2)
                print("vecs")

                print(vecs)

                # CALC ANGLES
                [x, y, q] = BaseController().get_current_pose()

                angles = []
                print(vecs)
                for i1 in len(vecs):
                    angles.append(acos((vecs[i1] - np.array([x, y]))) / (
                                np.linalg.norm(vecs[i1]) * np.linalg.norm(np.array([x, y]))))

                distances = getPoseDiff(locs[1], x, y)
                rospy.logwarn("dis ANG")
                rospy.logwarn(min(angles))
                rospy.logwarn(distances[angles.index(min(angles))])

                # FACE PERSON
                if min(angles) < ANGLE_THRESHOLD and distances[angles.index(min(angles))] < REACHING_THRESHOLDS:
                    headPoint = locs[angles.index(min(angles))]
                    found = True

    print(headPoint)
    BaseController().sync_face_to(headPoint.x, headPoint.y)
    return TdrResponse()


rospy.init_node("objectEngagementTracking")
rospy.Service('v2', Tdr, v2)
rospy.spin()