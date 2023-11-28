import rospy
import numpy as np

from PIL import Image
from sensor_msgs.msg import Image as SensorImage


def cv2_img_to_msg(img, stamp=None):
    """
    Convert a given cv2 image to sensor image

    :param img: cv2 image (as numpy array)
    :param stamp: rospy Time
    :return: Sensor Image
    """

    height, width, _ = img.shape

    msg = SensorImage()
    msg.header.stamp = rospy.Time.now() if stamp is None else stamp
    msg.width = width
    msg.height = height
    msg.encoding = 'bgr8'
    msg.is_bigendian = 1
    msg.step = 3 * width
    msg.data = img.tobytes()

    return msg


def msg_to_pillow_img(msg: SensorImage):
    """
    Convert a given sensor image to a pillow image

    :param msg: Sensor Image
    :return: Pillow Image
    """

    size = (msg.width, msg.height)
    if msg.encoding in ['bgr8', '8UC3']:
        img = Image.frombytes('RGB', size, msg.data, 'raw')

        # BGR => RGB
        img = Image.fromarray(np.array(img)[:, :, ::-1])
    elif msg.encoding == 'rgb8':
        img = Image.frombytes('RGB', size, msg.data, 'raw')
    else:
        raise Exception("Unsupported format.")

    return img


def msg_to_cv2_img(msg: SensorImage):
    """
    Convert a given sensor image to a cv2 image

    :param msg: Sensor Image
    :return: numpy array
    """

    img = msg_to_pillow_img(msg)

    # now bring it back into OpenCV format
    img = np.array(img)
    img = img[:, :, ::-1].copy()

    return img


def extract_mask_region(frame, mask, expand_x=0.5, expand_y=0.5):
    """
    Extracts the face region from the image and expands the region by the specified amount.

    :param frame: The source image.
    :param mask: The mask with the face part.
    :param expand_x: The percentage to expand the width of the bounding box.
    :param expand_y: The percentage to expand the height of the bounding box.
    :return: The extracted face region as a numpy array, or None if not found.
    """
    # only requiring cv2 if we need it
    import cv2

    contours, _ = cv2.findContours(
        mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest_contour)

        # Expand the bounding box
        new_w = w * (1 + expand_x)
        new_h = h * (1 + expand_y)
        x -= (new_w - w) // 2
        y -= (new_h - h) // 2

        # Ensure the new bounding box is within the frame dimensions
        x = int(max(0, x))
        y = int(max(0, y))
        new_w = min(frame.shape[1] - x, new_w)
        new_h = min(frame.shape[0] - y, new_h)

        face_region = frame[y:y+int(new_h), x:x+int(new_w)]
        return face_region
    return None
