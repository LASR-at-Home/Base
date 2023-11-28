#!/usr/bin/env python3

##
# WARNING: IM STILL CLEANING UP THIS CODE
# - paul
##

import cv2
import rospy
import smach
import cv2_img
import numpy as np

from sensor_msgs.msg import Image
from PIL import Image as PillowImage
from colour_estimation import closest_colours, RGB_COLOURS
from lasr_vision_msgs.msg import BodyPixMaskRequest, ColourPrediction, FeatureWithColour
from lasr_vision_msgs.srv import YoloDetection, BodyPixDetection, TorchFaceFeatureDetection


# BEGIN COPY PASTE

def extract_mask_region(frame, mask, expand_x=0.5, expand_y=0.5):
    """
    Extracts the face region from the image and expands the region by the specified amount.

    :param frame: The source image.
    :param mask: The mask with the face part.
    :param expand_x: The percentage to expand the width of the bounding box.
    :param expand_y: The percentage to expand the height of the bounding box.
    :return: The extracted face region as a numpy array, or None if not found.
    """
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


# END COPY PASTE

class TestDescribePeople(smach.StateMachine):

    def __init__(self):
        smach.StateMachine.__init__(
            self, outcomes=['succeeded', 'failed'], input_keys=[], output_keys=['people'])

        with self:
            smach.StateMachine.add('GET_IMAGE', self.GetImage(), transitions={
                                   'succeeded': 'SEGMENT'})
            # smach.StateMachine.add('RESIZE_TEST', self.ResizeTest(), transitions={'succeeded': 'SEGMENT_FACE'})
            # smach.StateMachine.add('SEGMENT_FACE', self.SegmentFace())

            sm_con = smach.Concurrence(outcomes=['succeeded', 'failed'],
                                       default_outcome='failed',
                                       outcome_map={'succeeded': {
                                           'SEGMENT_YOLO': 'succeeded', 'SEGMENT_BODYPIX': 'succeeded'}},
                                       input_keys=['img_msg'],
                                       output_keys=['people_detections', 'masks'])

            with sm_con:
                smach.Concurrence.add('SEGMENT_YOLO', self.SegmentYolo())
                smach.Concurrence.add('SEGMENT_BODYPIX', self.SegmentBodypix())

            smach.StateMachine.add('SEGMENT', sm_con, transitions={
                                   'succeeded': 'FILTER'})
            # smach.StateMachine.add('FILTER', self.Filter())
            # Uncomment for segment face:
            # smach.StateMachine.add('FILTER', self.Filter(), transitions={'succeeded': 'SEGMENT_FACE'})
            smach.StateMachine.add('FILTER', self.Filter(), transitions={
                                   'succeeded': 'succeeded'})
            # smach.StateMachine.add('RESIZE_TEST', self.ResizeTest(), transitions={'succeeded': 'SEGMENT_FACE'})
            # smach.StateMachine.add('SEGMENT_FACE', self.SegmentFace())

    # TODO: should be its own skill
    #       (could use ROS_MASTER_URI to determine if /camera/image_raw or /xtion/rgb/image_raw)
    class GetImage(smach.State):
        def __init__(self):
            smach.State.__init__(
                self, outcomes=['succeeded'], output_keys=['img_msg'])

        def execute(self, userdata):
            # userdata.img_msg = rospy.wait_for_message("/xtion/rgb/image_raw", Image)
            userdata.img_msg = rospy.wait_for_message(
                "/camera/image_raw", Image)
            return 'succeeded'

    class SegmentYolo(smach.State):
        def __init__(self):
            smach.State.__init__(self, outcomes=['succeeded', 'failed'], input_keys=[
                                 'img_msg'], output_keys=['people_detections'])
            self.yolo = rospy.ServiceProxy('/yolov8/detect', YoloDetection)

        def execute(self, userdata):
            try:
                result = self.yolo(
                    userdata.img_msg, "yolov8n-seg.pt", 0.5, 0.3)
                userdata.people_detections = [
                    det for det in result.detected_objects if det.name == "person"]
                return 'succeeded'
            except rospy.ServiceException as e:
                rospy.logwarn(f"Unable to perform inference. ({str(e)})")
                return 'failed'

    class SegmentBodypix(smach.State):
        def __init__(self):
            smach.State.__init__(self, outcomes=['succeeded', 'failed'], input_keys=[
                                 'img_msg'], output_keys=['masks'])
            self.bodypix = rospy.ServiceProxy(
                '/bodypix/detect', BodyPixDetection)

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

    class SegmentFace(smach.State):
        def __init__(self):
            smach.State.__init__(self, outcomes=['succeeded', 'failed'], input_keys=[
                                 'img_msg', 'people_detections', 'masks'], output_keys=[])
            self.torch_face_features = rospy.ServiceProxy(
                '/torch/detect/face_features', TorchFaceFeatureDetection)
            self.test = rospy.Publisher('/test', Image)

        def execute(self, userdata):
            try:

                # uncomment to make everything work as previously:
                # self.torch(userdata.img_msg, "resnet50", 0.7, [])

                # TODO: remove
                rospy.loginfo('Decoding')
                size = (userdata.img_msg.width, userdata.img_msg.height)
                if userdata.img_msg.encoding in ['bgr8', '8UC3']:
                    img = PillowImage.frombytes(
                        'RGB', size, userdata.img_msg.data, 'raw')

                    # BGR => RGB
                    img = PillowImage.fromarray(np.array(img)[:, :, ::-1])
                elif userdata.img_msg.encoding == 'rgb8':
                    img = PillowImage.frombytes(
                        'RGB', size, userdata.img_msg.data, 'raw')
                else:
                    raise Exception("Unsupported format.")

                frame = np.array(img)
                frame = frame[:, :, ::-1].copy()

                for person in userdata.people_detections:
                    # print(f"\n\nFound person with confidence {person.confidence}!")

                    # mask
                    mask_image = np.zeros((size[1], size[0]), np.uint8)
                    contours = np.array(person.xyseg).reshape(-1, 2)
                    cv2.fillPoly(mask_image, pts=np.int32(
                        [contours]), color=(255, 255, 255))
                    # mask_bin = mask_image > 128

                    # crop out face
                    face_mask = np.array(userdata.masks[1].mask).reshape(
                        userdata.masks[1].shape[0], userdata.masks[1].shape[1])
                    mask_image[face_mask == 0] = 0

                    a = extract_mask_region(frame, mask_image)
                    height, width, _ = a.shape

                    # a[mask_bin == 1] = 0

                    # print(a.shape)
                    # print(extract_mask_region(a, mask_image).shape)

                    # y,x = a.nonzero()
                    # minx = np.min(x)
                    # miny = np.min(y)
                    # maxx = np.max(x)
                    # maxy = np.max(y)

                    # a = a[miny:maxy, minx:maxx]
                    # print(a.shape)

                    msg = Image()
                    msg.header.stamp = rospy.Time.now()
                    msg.width = width
                    msg.height = height
                    msg.encoding = 'bgr8'
                    msg.is_bigendian = 1
                    msg.step = 3 * width
                    msg.data = a.tobytes()
                    self.test.publish(msg)
                    print(self.torch_face_features(msg))

                # jgifdjgfd

                return 'succeeded'
            except rospy.ServiceException as e:
                rospy.logwarn(f"Unable to perform inference. ({str(e)})")
                return 'failed'

    class ResizeTest(smach.State):
        def __init__(self):
            smach.State.__init__(self, outcomes=['succeeded', 'failed'], input_keys=[
                                 'img_msg'], output_keys=['img_msg'])

        def execute(self, userdata):
            # temp
            rospy.loginfo('Decoding')
            size = (userdata.img_msg.width, userdata.img_msg.height)
            if userdata.img_msg.encoding in ['bgr8', '8UC3']:
                img = PillowImage.frombytes(
                    'RGB', size, userdata.img_msg.data, 'raw')

                # BGR => RGB
                img = PillowImage.fromarray(np.array(img)[:, :, ::-1])
            elif userdata.img_msg.encoding == 'rgb8':
                img = PillowImage.frombytes(
                    'RGB', size, userdata.img_msg.data, 'raw')
            else:
                raise Exception("Unsupported format.")

            frame = np.array(img)
            frame = frame[:, :, ::-1].copy()
            frame = cv2.resize(frame, (128, 128))

            msg = Image()
            msg.header.stamp = rospy.Time.now()
            msg.width = 128
            msg.height = 128
            msg.encoding = 'bgr8'
            msg.is_bigendian = 1
            msg.step = 3 * 128
            msg.data = frame.tobytes()

            userdata.img_msg = msg
            return 'succeeded'

    class Filter(smach.State):
        def __init__(self):
            smach.State.__init__(self, outcomes=['succeeded', 'failed'], input_keys=[
                                 'img_msg', 'people_detections', 'masks'], output_keys=['people'])
            self.torch_face_features = rospy.ServiceProxy(
                '/torch/detect/face_features', TorchFaceFeatureDetection)

        def execute(self, userdata):
            if len(userdata.people_detections) == 0:
                print("Couldn't find anyone!")
                return 'failed'
            elif len(userdata.people_detections) == 1:
                print("There is one person.")
            else:
                print(f"There are {len(userdata.people_detections)} people.")

            # decode the image
            rospy.loginfo('Decoding')
            img = cv2_img.msg_to_cv2_img(userdata.img_msg)
            height, width, _ = img.shape

            people = []

            for person in userdata.people_detections:
                print(f"\n\nFound person with confidence {person.confidence}!")

                # mask
                mask_image = np.zeros((height, width), np.uint8)
                contours = np.array(person.xyseg).reshape(-1, 2)
                cv2.fillPoly(mask_image, pts=np.int32(
                    [contours]), color=(255, 255, 255))
                mask_bin = mask_image > 128

                # keep track
                features = []

                # process part masks
                for (bodypix_mask, part) in zip(userdata.masks, ['torso', 'head']):
                    part_mask = np.array(bodypix_mask.mask).reshape(
                        bodypix_mask.shape[0], bodypix_mask.shape[1])

                    # filter out part for current person segmentation
                    try:
                        part_mask[mask_bin == 0] = 0
                    except Exception:
                        print('|> Failed to check {part} is visible')
                        continue

                    if part_mask.any():
                        print(f'|> Person has {part} visible')
                    else:
                        print(f'|> Person does not have {part} visible')
                        continue

                    # TODO: construct userdata and not the stuff below!

                    # do colour processing on the torso
                    # TODO: this should actually be a separate state
                    if part == 'torso':
                        try:
                            features.append(FeatureWithColour("torso", [
                                ColourPrediction(colour, distance)
                                for colour, distance
                                in closest_colours(np.median(img[part_mask == 1], axis=0), RGB_COLOURS)
                            ]))
                        except Exception:
                            print("Failed to process colour")

                    # TODO: additional processing through Ben's model
                    if part == 'head':
                        # use img[part_mask == 1]
                        pass

                # THIS IS FROM SegmentFace state but I am lazy for now because i dont have neough time to clean this up properly right now ok
                mask_image = np.zeros((height, width), np.uint8)
                contours = np.array(person.xyseg).reshape(-1, 2)
                cv2.fillPoly(mask_image, pts=np.int32(
                    [contours]), color=(255, 255, 255))
                # mask_bin = mask_image > 128

                # crop out face
                face_mask = np.array(userdata.masks[1].mask).reshape(
                    userdata.masks[1].shape[0], userdata.masks[1].shape[1])
                mask_image[face_mask == 0] = 0

                a = extract_mask_region(img, mask_image)
                if a is None:
                    print("Failed to detect face features")
                    continue

                height, width, _ = a.shape

                # TODO: common method
                msg = Image()
                msg.header.stamp = rospy.Time.now()
                msg.width = width
                msg.height = height
                msg.encoding = 'bgr8'
                msg.is_bigendian = 1
                msg.step = 3 * width
                msg.data = a.tobytes()
                features.extend(self.torch_face_features(
                    msg).detected_features)

                people.append({
                    'detection': person,
                    'features': features
                })

            # TODO: construct userdata like:
            # - people
            #   - - detection (YOLO)
            #     - parts
            #       - - part
            #         - mask

            userdata['people'] = people
            return 'succeeded'
