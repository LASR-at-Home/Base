#!/usr/bin/env python3

import rospy
from lasr_object_detection_yolo.srv import YoloDetection, YoloDetectionRequest
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
    print("response")
    print(response)
    return np.array([response.target_point.point.x, response.target_point.point.y, response.target_point.point.z])
def perform_detection(yolo,tf, bridge, shapely, pcl_msg, polygon, filter, model="yolov8n-seg.pt"):
    cv_im = pcl_msg_to_cv2(pcl_msg)
    img_msg = bridge.cv2_to_imgmsg(cv_im)
    detections = yolo(img_msg, model, 0.5, 0.3)
    rospy.loginfo(detections)
    detections = [(det, estimate_pose(tf, pcl_msg, det)) for det in detections.detected_objects if
                  det.name in filter]
    rospy.loginfo(f"All: {[(det.name, pose) for det, pose in detections]}")
    rospy.loginfo(f"Boundary: {polygon}")
    satisfied_points = shapely.are_points_in_polygon_2d(polygon, [[pose[0], pose[1]] for (_, pose) in
                                                                               detections]).inside
    detections = [detections[i] for i in range(0, len(detections)) if satisfied_points[i]]
    rospy.loginfo(f"Filtered: {[(det.name, pose) for det, pose in detections]}")
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

    print("now calling perform detection")
    pcl_msg = rospy.wait_for_message("/xtion/depth_registered/points", PointCloud2)
    polygon = rospy.get_param('test_lift_points')
    detections, im = perform_detection(yolo, tf, bridge, shapely, pcl_msg, polygon, ["person"], "yolov8n-seg.pt")
    debug(im, detections)
    print(detections)
    rospy.spin()

