import rospy
import cv2_img
import numpy as np

from PIL import Image
from ultralytics import YOLO

from sensor_msgs.msg import Image as SensorImage
from lasr_vision_msgs.msg import Detection
from lasr_vision_msgs.srv import YoloDetectionRequest, YoloDetectionResponse

# model cache
loaded_models = {}

def load_model(dataset: str) -> None:
    '''
    Load a model into cache
    '''

    model = None
    if dataset in loaded_models:
        model = loaded_models[dataset]
    else:
        model = YOLO(dataset)
        rospy.loginfo(f'Loaded {dataset} model')

        loaded_models[dataset] = model
    
    return model

def detect(request: YoloDetectionRequest, debug_publisher: rospy.Publisher | None) -> YoloDetectionResponse:
    '''
    Run YOLO inference on given detection request
    '''

    # decode the image
    rospy.loginfo('Decoding')
    img = cv2_img.msg_to_pillow_img(request.image_raw)
    
    # load model
    rospy.loginfo('Loading model')
    model = load_model(request.dataset)

    # run inference
    rospy.loginfo('Running inference')
    results = model(img, conf=request.confidence, iou=request.nms)
    result = results[0]
    rospy.loginfo('Inference complete')

    # construct response
    detected_objects = []
    object_count = result.boxes.cls.size(dim=0)
    has_segment_masks = result.masks is not None
    for i in range(0, object_count):
        detection = Detection()
        detection.name = result.names[int(result.boxes.cls[i])]
        detection.confidence = float(result.boxes.conf.cpu().numpy()[i])
        detection.xywh = result.boxes.xywh[i].cpu().numpy().astype(int).tolist()

        # copy segmented mask if available
        if has_segment_masks:
            detection.xyseg = result.masks.xy[i].flatten().astype(int).tolist()

        detected_objects.append(detection)
    
    # publish to debug topic
    if debug_publisher is not None:
        debug_publisher.publish(cv2_img.cv2_img_to_msg(result.plot()))
    
    response = YoloDetectionResponse()
    response.detected_objects = detected_objects
    return response
