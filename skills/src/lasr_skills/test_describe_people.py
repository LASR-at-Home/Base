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

from .vision import GetImage


class TestDescribePeople(smach.StateMachine):

    def __init__(self):
        smach.StateMachine.__init__(
            self, outcomes=['succeeded', 'failed'], input_keys=[], output_keys=['people'])

        with self:
            smach.StateMachine.add('GET_IMAGE', GetImage(), transitions={
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

                    a = cv2_img.extract_mask_region(frame, mask_image)
                    height, width, _ = a.shape

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
            frame = cv2_img.msg_to_cv2_img(userdata.img_msg)
            frame = cv2.resize(frame, (128, 128))
            userdata.img_msg = cv2_img.cv2_img_to_msg(frame)
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

                a = cv2_img.extract_mask_region(img, mask_image)
                if a is None:
                    print("Failed to detect face features")
                    continue

                msg = cv2_img.cv2_img_to_msg(a)
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
