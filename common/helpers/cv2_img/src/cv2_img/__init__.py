import rospy
import numpy as np

from PIL import Image
from sensor_msgs.msg import Image as SensorImage

def cv2_img_to_msg(img, stamp = None):
    '''
    Convert a given cv2 image to sensor image
    '''

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
    '''
    Convert a given sensor image to a pillow image
    '''

    size = (msg.width, msg.height)
    if msg.encoding in ['bgr8', '8UC3']:
        img = Image.frombytes('RGB', size, msg.data, 'raw')

        # BGR => RGB
        img = Image.fromarray(np.array(img)[:,:,::-1])
    elif msg.encoding == 'rgb8':
        img = Image.frombytes('RGB', size, msg.data, 'raw')
    else:
        raise Exception("Unsupported format.")

    return img

def msg_to_cv2_img(msg: SensorImage):
    '''
    Convert a given sensor image to a cv2 image
    '''

    img = msg_to_pillow_img(msg)
    
    # now bring it back into OpenCV format
    img = np.array(img)
    img = img[:, :, ::-1].copy()

    return img
