#!/usr/bin/env python3
import cv2
import rospy
import smach
import numpy as np

from sensor_msgs.msg import Image
from PIL import Image as PillowImage
from lasr_vision_msgs.msg import BodyPixMaskRequest
from lasr_vision_msgs.srv import YoloDetection, BodyPixDetection

# ALTERNATIVE

# Dictionary of CIELAB colours
LAB_colours = {
    "black" : (0.0, 0.0),
    "white" : (0.0, 0.0),
    "grey" : (0.0, 0.0),
    "blue" : (68.0, -112.0),
    "yellow": (-15.0, 93.0),
    "red": (80.0, 70.0),
    "green": (-47.0, 48.0),
    "orange": (27.0, 79.0),
    "purple": (75.0, -96.0),
    "brown": (50.0, 31.0)
}

def quantize_image(image, k):
    """
    Apply quantization to image, in order to
    reduce number of distinct colours.
    Parameters
    ----------
    image : cv2 image
        Target image for qunatization
    k : int
        Number of clusters for k-means.
    Returns
    -------
    cv2 image
        Quantized image.
    """
    # Flatten image, but keep colour channels.
    img = np.float32(image).reshape(-1,3)
    # Apply k-means
    condition = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,20,1.0)
    ret,label,center = cv2.kmeans(img, k , None, condition,10,cv2.KMEANS_RANDOM_CENTERS)
    center = np.uint8(center)
    return center[label.flatten()].reshape(image.shape)

def closest_colour(colour):
    """
    Determine closest 'text' colour to given colour.
    Parameters
    ----------
    colour : tuple(b, g, r)
        Input colour
    Returns
    -------
    str : 'text' colour
        Closest 'text' colour
    """
    # Convert BGR to LAB
    l, a, b = cv2.cvtColor(np.array([[colour]], np.uint8), cv2.COLOR_BGR2LAB)[0][0]

    # Convert OpenCV LAB to CIELAB
    l*=100//255
    a//=2
    b = 127 - b//2

    # Use euclidian distance to determine the best fit
    # for the colour.
    colour_distances = {k : [0, 0] for k in LAB_colours.keys()}
    for k in colour_distances.keys():
        colour_distances[k][0] = np.linalg.norm(a - LAB_colours[k][0])
        colour_distances[k][1] = np.linalg.norm(b - LAB_colours[k][1])
    colour_name = list(LAB_colours.keys())[np.argmin([sum(v) for v in colour_distances.values()])]

    # Prepend prefix from perceptual lightness
    if l < 30:
        prefix = 'light '
    elif l > 70:
        prefix = 'dark '
    else:
        prefix = ''

    return prefix + colour_name

def dominant_colour(img):
    """
    Determines the dominant colour in an image.
    Parameters
    ----------
    img : cv2 image
        Input image
    Returns
    -------
    str : colour 'text'
        Text representation of colour.
    """
    img = quantize_image(img, 9)
    colours, count = np.unique(img.reshape(-1,img.shape[-1]), axis=0, return_counts=True)
    dominant = colours[np.argmax(count)]
    return closest_colour(dominant)
#END ALTERNATIVE


# START COPY PASTE
COLORS = {
        "red": [255, 0, 0],
        "green": [0, 255, 0],
        "blue": [0, 0, 255],
        "white": [255, 255, 255],
        "black": [0, 0, 0],
        "yellow": [255, 255, 0],
        "cyan": [0, 255, 255],
        "magenta": [255, 0, 255],
        "gray": [128, 128, 128],
        "orange": [255, 165, 0],
        "purple": [128, 0, 128],
        "brown": [139, 69, 19],
        "pink": [255, 182, 193],
        "beige": [245, 245, 220],
        "maroon": [128, 0, 0],
        "olive": [128, 128, 0],
        "navy": [0, 0, 128],
        "lime": [50, 205, 50],
        "golden": [255, 223, 0],
        "teal": [0, 128, 128],
        "coral": [255, 127, 80],
        "salmon": [250, 128, 114],
        "turquoise": [64, 224, 208],
        "violet": [238, 130, 238],
        "platinum": [229, 228, 226],
        "ochre": [204, 119, 34],
        "burntsienna": [233, 116, 81],
        "chocolate": [210, 105, 30],
        "tan": [210, 180, 140],
        "ivory": [255, 255, 240],
        "goldenrod": [218, 165, 32],
        "orchid": [218, 112, 214],
        "honey": [238, 220, 130]
    }

def closest_colours(requested_color):
    distances = {color: np.linalg.norm(np.array(rgb_val) - requested_color) for color, rgb_val in COLORS.items()}
    sorted_colors = sorted(distances.items(), key=lambda x: x[1])
    top_three_colors = sorted_colors[:3]
    formatted_colors = [(color_name, distance) for color_name, distance in top_three_colors]
    return formatted_colors

# END COPY PASTE

class TestDescribePeople(smach.StateMachine):

    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'failed'], input_keys=['img_msg'], output_keys=['people_detections'])

        with self:
            smach.StateMachine.add('GET_IMAGE', self.GetImage(), transitions={'succeeded' : 'SEGMENT'})

            sm_con = smach.Concurrence(outcomes=['succeeded','failed'],
                                       default_outcome='failed',
                                       outcome_map={'succeeded': { 'SEGMENT_YOLO': 'succeeded', 'SEGMENT_BODYPIX': 'succeeded' }},
                                       input_keys=['img_msg'],
                                       output_keys=['people_detections', 'masks'])
            
            with sm_con:
                smach.Concurrence.add('SEGMENT_YOLO', self.SegmentYolo())
                smach.Concurrence.add('SEGMENT_BODYPIX', self.SegmentBodypix())

            smach.StateMachine.add('SEGMENT', sm_con, transitions={'succeeded': 'FILTER'})
            smach.StateMachine.add('FILTER', self.Filter())

            # smach.StateMachine.add('DETECT_PEOPLE', DetectPeople(), transitions={'succeeded' : 'CHECK_FOR_PERSON', 'failed' : 'failed'})
            # smach.StateMachine.add('CHECK_FOR_PERSON', self.CheckForPerson(), transitions={'done' : 'succeeded', 'not_done' : 'GET_IMAGE'})

    # TODO: should be its own skill
    class GetImage(smach.State):
        def __init__(self):
            smach.State.__init__(self, outcomes=['succeeded'], output_keys=['img_msg'])
        
        def execute(self, userdata):
            # userdata.img_msg = rospy.wait_for_message("/xtion/rgb/image_raw", Image)
            userdata.img_msg = rospy.wait_for_message("/camera/image_raw", Image)
            return 'succeeded'

    class SegmentYolo(smach.State):
        def __init__(self):
            smach.State.__init__(self, outcomes=['succeeded', 'failed'], input_keys=['img_msg'], output_keys=['people_detections'])
            self.yolo = rospy.ServiceProxy('/yolov8/detect', YoloDetection)

        def execute(self, userdata):
            try:
                result = self.yolo(userdata.img_msg, "yolov8n-seg.pt", 0.5, 0.3)
                userdata.people_detections = [det for det in result.detected_objects if det.name == "person"]
                return 'succeeded'
            except rospy.ServiceException as e:
                rospy.logwarn(f"Unable to perform inference. ({str(e)})")
                return 'failed'

    class SegmentBodypix(smach.State):
        def __init__(self):
            smach.State.__init__(self, outcomes=['succeeded', 'failed'], input_keys=['img_msg'], output_keys=['masks'])
            self.bodypix = rospy.ServiceProxy('/bodypix/detect', BodyPixDetection)

        def execute(self, userdata):
            try:
                torso = BodyPixMaskRequest()
                torso.parts = ["torso_front", "torso_back"]

                head = BodyPixMaskRequest()
                head.parts = ["left_face", "right_face"]

                masks = [torso, head]

                result = self.bodypix(userdata.img_msg, "resnet50", 0.7, masks)
                userdata.masks = result.masks
                return 'succeeded'
            except rospy.ServiceException as e:
                rospy.logwarn(f"Unable to perform inference. ({str(e)})")
                return 'failed'

    class Filter(smach.State):
        def __init__(self):
            smach.State.__init__(self, outcomes=['succeeded', 'failed'], input_keys=['img_msg', 'people_detections', 'masks'], output_keys=[])

        def execute(self, userdata):
            if len(userdata.people_detections) == 0:
                print("Couldn't find anyone!")
                return 'failed'
            elif len(userdata.people_detections) == 1:
                print("There is one person.")
            else:
                print(f"There are {len(userdata.people_detections)} people.")

            # BEGIN COPY PASTE ===========================
            # decode the image
            # TODO: turn this into a common utility
            rospy.loginfo('Decoding')
            size = (userdata.img_msg.width, userdata.img_msg.height)
            if userdata.img_msg.encoding in ['bgr8', '8UC3']:
                img = PillowImage.frombytes('RGB', size, userdata.img_msg.data, 'raw')

                # BGR => RGB
                img = PillowImage.fromarray(np.array(img)[:,:,::-1])
            elif userdata.img_msg.encoding == 'rgb8':
                img = PillowImage.frombytes('RGB', size, userdata.img_msg.data, 'raw')
            else:
                raise Exception("Unsupported format.")
            
            # now bring it back into OpenCV format
            img = np.array(img)
            img = img[:, :, ::-1].copy() 
            # END COPY PASTE ===========================

            height, width, _ = img.shape

            for person in userdata.people_detections:
                print(f"Found person with confidence {person.confidence}!")

                # mask
                mask_image = np.zeros((height, width), np.uint8)
                contours = np.array(person.xyseg).reshape(-1, 2)
                cv2.fillPoly(mask_image, pts = np.int32([contours]), color = (255,255,255))
                mask_bin = mask_image > 128

                # process part masks
                for (bodypix_mask, part) in zip(userdata.masks, ['torso', 'head']):
                    part_mask = np.array(bodypix_mask.mask).reshape(bodypix_mask.shape[0], bodypix_mask.shape[1])

                    # filter out part for current person segmentation
                    part_mask[mask_bin == 0] = 0

                    if part_mask.any():
                        print(f'|> Person has {part} visible')
                    else:
                        print(f'|> Person does not have {part} visible')
                        continue

                    # TODO: construct userdata and not the stuff below!

                    # do colour processing on the torso
                    # TODO: this should actually be a separate state
                    if part == 'torso':
                        print(closest_colours(np.median(img[part_mask == 1], axis=0)))
                        print(dominant_colour(img[part_mask == 1]))

                    # TODO: additional processing through Ben's model
                    if part == 'head':
                        # use img[part_mask == 1]
                        pass

            # TODO: construct userdata like:
            # - people
            #   - - parts
            #       - - part
            #         - img

            return 'succeeded'
