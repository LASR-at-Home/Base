#!/usr/bin/env python3

import rospy
from lasr_vision_msgs.srv import YoloDetection, YoloDetectionRequest
from sensor_msgs.msg import Image
from common_math import pcl_msg_to_cv2, seg_to_centroid
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped, Point
from tf_module.srv import TfTransform, TfTransformRequest
import numpy as np
from lasr_shapely import LasrShapely
from cv_bridge3 import CvBridge
from sensor_msgs.msg import PointCloud2
import cv2
# from tiago_controllers.helpers.nav_map_helpers import is_close_to_object, rank



def detect_objects(object_names: [str], confidence=0.25, nms=0.4, model="yolov8n.pt"):
    """
    Detects all persons in an image using yolov8
    """
    rospy.wait_for_service("/yolov8/detect", rospy.Duration(15.0))

    if not isinstance(object_names, list):
        raise ValueError("please input a list of strings")

    try:
        detect_service = rospy.ServiceProxy('/yolov8/detect', YoloDetection)

        if rospy.get_published_topics(namespace='/camera/image_raw'):
            image_msg = rospy.wait_for_message('/camera/image_raw', Image)  # wait for depth image
        elif rospy.get_published_topics(namespace='/xtion/rgb/image_raw'):
            image_msg = rospy.wait_for_message('/xtion/rgb/image_raw', Image)
        elif rospy.get_published_topics(namespace='/usb_cam/image_raw'):
            image_msg = rospy.wait_for_message('/usb_cam/image_raw', Image)  # wait for rgb image


        req = YoloDetectionRequest()
        req.image_raw = image_msg
        req.dataset = model
        req.confidence = confidence
        req.nms = nms
        resp = detect_service(req)
        print(resp)

    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)
        return []

    objects = []
    for obj in resp.detected_objects:
        if obj.name in object_names:
            objects.append(obj)
    return objects

def estimate_pose(tf, pcl_msg, detection):
    centroid_xyz = seg_to_centroid(pcl_msg, np.array(detection.xyseg))
    centroid = PointStamped()
    centroid.point = Point(*centroid_xyz)
    centroid.header = pcl_msg.header
    tf_req = TfTransformRequest()
    tf_req.target_frame = String("map")
    tf_req.point = centroid
    response = tf(tf_req)
    # print("response")
    # print(response)
    return np.array([response.target_point.point.x, response.target_point.point.y, response.target_point.point.z])
# def perform_detection(yolo,tf, bridge, shapely, pcl_msg, polygon, filter, model="yolov8n-seg.pt"):
#     cv_im = pcl_msg_to_cv2(pcl_msg)
#     img_msg = bridge.cv2_to_imgmsg(cv_im)
#     detections = yolo(img_msg, model, 0.5, 0.3)
#     rospy.loginfo(detections)
#     detections = [(det, estimate_pose(tf, pcl_msg, det)) for det in detections.detected_objects if
#                   det.name in filter]
#     rospy.loginfo(f"All: {[(det.name, pose) for det, pose in detections]}")
#     rospy.loginfo(f"Boundary: {polygon}")
#     satisfied_points = shapely.are_points_in_polygon_2d(polygon, [[pose[0], pose[1]] for (_, pose) in
#                                                                                detections]).inside
#     detections = [detections[i] for i in range(0, len(detections)) if satisfied_points[i]]
#     rospy.loginfo(f"Filtered: {[(det.name, pose) for det, pose in detections]}")
#     print(len(detections))
#     return detections, img_msg


# for the comp
def perform_detection(default, pcl_msg, polygon, filter, model="yolov8n-seg.pt"):
    cv_im = pcl_msg_to_cv2(pcl_msg)
    img_msg = default.bridge.cv2_to_imgmsg(cv_im)
    detections = default.yolo(img_msg, model, 0.5, 0.3)
    # rospy.loginfo(detections)
    detections = [(det, estimate_pose(default.tf, pcl_msg, det)) for det in detections.detected_objects if
                  det.name in filter]

    # rospy.loginfo(f"All: {[(det.name, pose) for det, pose in detections]}")
    # rospy.loginfo(f"Boundary: {polygon}")
    if polygon is None:
        return detections, img_msg
    else:
        satisfied_points = default.shapely.are_points_in_polygon_2d(polygon, [[pose[0], pose[1]] for (_, pose) in
                                                                              detections]).inside
        detections = [detections[i] for i in range(0, len(detections)) if satisfied_points[i]]
    # rospy.loginfo(f"Filtoprintered: {[(det.name, pose) for det, pose in detections]}")
    print("len detections --->")
    print(len(detections))
    return detections, img_msg


def debug(image, resp):
    rospy.loginfo("Received image message")
    try:
        # pick the first detection as an example
        if len(resp) > 0:
            detection = resp[0][0]

            if len(detection.xyseg) > 0:
                # unflatten the array
                contours = np.array(detection.xyseg).reshape(-1, 2)

                # draw using opencv
                img = np.zeros((image.height, image.width), dtype=np.uint8)
                cv2.fillPoly(img, pts = [contours], color = (255,255,255))

                # send to topic
                # img_msg = bridge.cv2_to_imgmsg(img, encoding="passthrough")
                # debug_publisher1.publish(img_msg)
                debug_publisher1.publish(image)
            else:
                print('WARN: No segmentation was performed on the image!')
        else:
            print('WARN: no detections')

        # draw all of them
        if len(resp) > 0:
            img = np.zeros((image.height, image.width), dtype=np.uint8)
            for detection in resp:
                detection = detection[0]
                if len(detection.xyseg) > 0:
                    contours = np.array(detection.xyseg).reshape(-1, 2)
                    r,g,b = np.random.randint(0, 255, size=3)
                    cv2.fillPoly(img, pts = [contours], color = (int(r), int(g), int(b)))
            img_msg = bridge.cv2_to_imgmsg(img, encoding="passthrough")
            debug_publisher2.publish(img_msg)
        else:
            print('WARN: no detections')

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

def is_anyone_in_front_of_me(yolo,tf, bridge, shapely, pcl_msg, polygon, filter, model="yolov8n-seg.pt"):
    pcl_msg = rospy.wait_for_message("/xtion/depth_registered/points", PointCloud2)
    polygon = rospy.get_param('lift_position_points')
    print(polygon)
    detections, im = perform_detection(yolo, tf, bridge, shapely, pcl_msg, polygon, filter, model="yolov8n-seg.pt")
    return len(detections) > 0

DIST_THRESH = 0.1

def euclidean_distance(point1, point2):
    return np.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2 + (point1[2] - point2[2]) ** 2)
# def match_detections_between_frames(det1, det2):
#     matches = {}
#
#     for i, position1 in enumerate(det1):
#         closest_distance = float('inf')
#         closest_position = None
#
#         for j, position2 in enumerate(det2):
#             distance = euclidean_distance(position1, position2)
#             if distance < closest_distance:
#                 closest_distance = distance
#                 closest_position = position2
#
#         if closest_position is not None:
#             matches[i] = closest_position
#
#     robot_position = [robot_x, robot_y, robot_z]
#
#     for i, position2 in matches.values():
#         vector = np.array(position2) - np.array(det1[i])
#
#         vector_to_robot = np.array(robot_position) - np.array(det1[i])
#         dot_product = np.dot(vector, vector_to_robot)
#
#         if dot_product > 0:
#             print(f"Position {i + 1} in frame 1 faces the robot.")
#         else:
#             print(f"Position {i + 1} in frame 1 does not face the robot.")
#
#     static_pos = []
#     moving_pos = []
#
#     for i, position2 in matches.items():
#         initial_position = det1[i]
#         final_position = position2
#
#         initial_distance_to_robot = euclidean_distance(initial_position, robot_position)
#         final_distance_to_robot = euclidean_distance(final_position, robot_position)
#
#         if final_distance_to_robot < initial_distance_to_robot:
#             # moved closer
#             moving_pos.append(i)
#         elif final_distance_to_robot > initial_distance_to_robot:
#             #mode further
#             pass
#         else:
#             # remained the same
#             static_pos.append(i)
#
#     # face the quat
#     return static_pos, moving_pos


def phase1(yolo, tf, bridge, shapely, pcl_msg, polygon, filter, model):
    # get two frames at the time
    pcl_msg = rospy.wait_for_message("/xtion/depth_registered/points", PointCloud2)
    polygon = rospy.get_param('test_lift_points')
    detections1, im = perform_detection(yolo, tf, bridge, shapely, pcl_msg, polygon, filter, model)
    rospy.sleep(5)
    pcl_msg = rospy.wait_for_message("/xtion/depth_registered/points", PointCloud2)
    detections2, im = perform_detection(yolo, tf, bridge, shapely, pcl_msg, polygon, filter, model)
    while len(detections1) != len(detections2):
        detections1, im = perform_detection(yolo, tf, bridge, shapely, pcl_msg, polygon, filter, model)
        rospy.sleep(5)
        pcl_msg = rospy.wait_for_message("/xtion/depth_registered/points", PointCloud2)
        detections2, im = perform_detection(yolo, tf, bridge, shapely, pcl_msg, polygon, filter, model)

    # match the detections based on euclidean distance
    matching = {}
    not_matching = []
    for i, det1 in enumerate(detections1):
        for j, det2 in enumerate(detections2):
            if np.linalg.norm(det1[1] - det2[1]) < DIST_THRESH:
                matching[i] = j
            else:
                not_matching.append(j)

    # print(not_matching)




    # if the





if __name__ == '__main__':
    rospy.init_node("objects_detection_yolov8", anonymous=True)

    # object detection yolo v8
    # objects = detect_objects(["person", "mug", "phone"])
    # for o in objects:
    #     print("object name: ", o.name)
    #     print("object confidence: ", o.confidence)
    #     print("object position: ", o.xywh)
    debug_publisher1 = rospy.Publisher('/yolov8/debug_mask', Image, queue_size=1)
    debug_publisher2 = rospy.Publisher('/yolov8/debug_mask_all', Image, queue_size=1)

    # perform detection and segmentation
    rospy.wait_for_service('/yolov8/detect')
    yolo = rospy.ServiceProxy('/yolov8/detect', YoloDetection)
    rospy.wait_for_service("/tf_transform")
    tf = rospy.ServiceProxy("/tf_transform", TfTransform)
    shapely = LasrShapely()
    rospy.loginfo("Got shapely")
    bridge = CvBridge()
    rospy.loginfo("CV Bridge")

    # yolo seg only
    # print("now calling perform detection")
    # pcl_msg = rospy.wait_for_message("/xtion/depth_registered/points", PointCloud2)
    # polygon = rospy.get_param('test_lift_points')
    # detections, im = perform_detection(yolo, tf, bridge, shapely, pcl_msg, polygon, ["person"], "yolov8n-seg.pt")
    # debug(im, detections)
    # print("now printing detections")
    # print(len(detections))
    # pos_people = []
    # for i, person in detections:
    #     print(person)
    #     pos_people.append([person[0], person[1]])
    #
    #     print(person[0], person[1])
    #
    # print(pos_people)
    # yolo seg only

    pcl_msg = rospy.wait_for_message("/xtion/depth_registered/points", PointCloud2)
    polygon = rospy.get_param('test_lift_points')
    detections, im = perform_detection(yolo, tf, bridge, shapely, pcl_msg, polygon, ["person"], "yolov8n-seg.pt")

    pos_people = []
    for i, person in detections:
        # print(person)
        person = person.tolist()
        # print(type(person))
        pos_people.append([person[0], person[1]])

    num_people = len(detections)

    rospy.set_param("/lift/num_people", num_people)
    rospy.set_param("/lift/pos_persons", pos_people)

    pcl_msg = rospy.wait_for_message("/xtion/depth_registered/points", PointCloud2)
    polygon = rospy.get_param('test_lift_points')
    # print("is anyone in front of me >")
    # print(is_anyone_in_front_of_me(yolo, tf, bridge, shapely, pcl_msg, polygon, ["person"], "yolov8n-seg.pt"))
    # print("is anyone in front of me")

    # detections of i and then th esecond from the tuple
    # print(detections[0][1])
    rospy.spin()

