#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan, Image
from lasr_perception_server.srv import DetectImage
import numpy as np
import cv2
import mediapipe as mp
from fer import FER
from object_interest_tracking.srv import EngagementScoreRequest, EngagementScoreResponse, ImgLstConverter, \
    ImgLstConverterRequest, EngagementScore
from PIL import Image as PILImage
from cv_bridge3 import CvBridge


def getData():
    # get image 
    img_msg = rospy.wait_for_message('/xtion/rgb/image_raw', Image)

    # get laser reading
    laser_scan = ""  # rospy.wait_for_message('/scan_raw', LaserScan)

    return img_msg, laser_scan


# def yawPitchRoll(faceMesh):
#     # yaw
#     print(faceMesh)

#     return 0, 0, 0


def emotionScore(score):
    a = {
        "angry": -5,
        "disgust": -4,
        "fear": -3,
        "happy": 5,
        "sad": 1,
        "surprise": 4,
        "neutral": 2,
    }

    return a[score]


def locateEngagedObjects(req):
    img, laserReading = getData()
    scores = {}

    # detect ppl -> img
    rospy.wait_for_service('lasr_perception_server/detect_objects_image')
    objectRecognitionService = rospy.ServiceProxy(
        'lasr_perception_server/detect_objects_image', DetectImage
    )

    resp = objectRecognitionService(
        [img], 'coco', 0.7, 0.3, ["person"], 'yolo'
    ).detected_objects

    # assign them to score map
    c = 0
    for i in resp:
        scores[c] = {"xywh": i.xywh, "score": 0}
        c += 1

    # calculate face/eyes direction -> img

    image2D = CvBridge().imgmsg_to_cv2_np(img)
    EncodedImage2D = cv2.cvtColor(image2D,
                                  cv2.COLOR_BGR2RGB)  # no need since _pnp will do it, here because will unuse _pnp
    EncodedImage2D.flags.writeable = False

    new_image = PILImage.fromarray(EncodedImage2D)
    new_image.save(f'newMTr.png')

    for i in scores:
        # detect face orientation
        r = EncodedImage2D[max(0, scores[i]["xywh"][1]):max(0, scores[i]["xywh"][1]) + scores[i]["xywh"][3] + 1,
            max(0, scores[i]["xywh"][0]):max(0, scores[i]["xywh"][0]) + scores[i]["xywh"][2] + 1]
        r = r.copy(order='c')
        # new_image = PILImage.fromarray(r)
        # new_image.save(f'ddq{i}.png')

        # BaseOptions = mp.tasks.BaseOptions
        # FaceLandmarker = mp.tasks.vision.FaceLandmarker
        # FaceLandmarkerOptions = mp.tasks.vision.FaceLandmarkerOptions
        # VisionRunningMode = mp.tasks.vision.RunningMode

        # options = FaceLandmarkerOptions(
        #     base_options=BaseOptions(model_asset_path=model_path),
        #     running_mode=VisionRunningMode.IMAGE)

        # with FaceLandmarker.create_from_options(options) as landmarker:
        #     face_landmarker_result = landmarker.detect(r)
        #     yaw, pitch, roll = yawPitchRoll(face_landmarker_result)

        # emotions -> img

        detector = FER()
        emotion = detector.top_emotion(r)
        scores[i]['score'] += emotionScore(emotion[0])

    print(scores[max(scores, key=lambda x: scores[x]['score'])])
    print(scores[max(scores, key=lambda x: scores[x]['score'])]['xywh'])
    res = EngagementScoreResponse()
    res.dimensions = list(scores[max(scores, key=lambda x: scores[x]['score'])]['xywh'])
    return res

    # distraction detection -> img & laser


rospy.init_node("objectEngagementTracking")
rospy.Service('engagementScore', EngagementScore, locateEngagedObjects)
rospy.spin()