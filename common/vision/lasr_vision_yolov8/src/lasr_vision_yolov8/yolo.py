import rospy
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
    size = (request.image_raw.width, request.image_raw.height)
    if request.image_raw.encoding in ['bgr8', '8UC3']:
        img = Image.frombytes('RGB', size, request.image_raw.data, 'raw')

        # BGR => RGB
        img = Image.fromarray(np.array(img)[:,:,::-1])
    elif request.image_raw.encoding == 'rgb8':
        img = Image.frombytes('RGB', size, request.image_raw.data, 'raw')
    else:
        raise Exception("Unsupported format.")
    
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
        msg = SensorImage()
        msg.header.stamp = rospy.Time.now()
        msg.width = request.image_raw.width
        msg.height = request.image_raw.height
        msg.encoding = 'bgr8'
        msg.is_bigendian = 1
        msg.step = 3 * request.image_raw.width
        msg.data = result.plot().tobytes()
        debug_publisher.publish(msg)
    
    response = YoloDetectionResponse()
    response.detected_objects = detected_objects
    return response
