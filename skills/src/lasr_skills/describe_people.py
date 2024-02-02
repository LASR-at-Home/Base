#!/usr/bin/env python3

import cv2
import rospy
import smach
import cv2_img
import numpy as np

# from colour_estimation import closest_colours, RGB_COLOURS
from lasr_vision_msgs.msg import BodyPixMaskRequest, ColourPrediction, FeatureWithColour
from lasr_vision_msgs.srv import YoloDetection, BodyPixDetection, TorchFaceFeatureDetection
from numpy2message import numpy2message

from .vision import GetImage, ImageMsgToCv2, Get3DImage, PclMsgToCv2, Get2DAnd3DImages


class DescribePeople(smach.StateMachine):

    def __init__(self):
        smach.StateMachine.__init__(
            self, outcomes=['succeeded', 'failed'], input_keys=[], output_keys=['people'])

        with self:
            # smach.StateMachine.add('GET_IMAGE', GetImage(), transitions={
            #                        'succeeded': 'CONVERT_IMAGE'})
            # smach.StateMachine.add('CONVERT_IMAGE', ImageMsgToCv2(), transitions={
            #                        'succeeded': 'SEGMENT'})

            smach.StateMachine.add('GET_IMAGE', Get2DAnd3DImages(), transitions={
                                   'succeeded': 'CONVERT_IMAGE'})
            smach.StateMachine.add('CONVERT_IMAGE', PclMsgToCv2(), transitions={
                                   'succeeded': 'SEGMENT'})

            sm_con = smach.Concurrence(outcomes=['succeeded', 'failed'],
                                       default_outcome='failed',
                                       outcome_map={'succeeded': {
                                           'SEGMENT_YOLO': 'succeeded', 'SEGMENT_BODYPIX': 'succeeded'}},
                                       input_keys=['img', 'img_msg_2d', 'img_msg_3d', 'xyz'],
                                       output_keys=['people_detections', 'bodypix_masks'])

            with sm_con:
                smach.Concurrence.add('SEGMENT_YOLO', self.SegmentYolo())
                smach.Concurrence.add('SEGMENT_BODYPIX', self.SegmentBodypix())

            smach.StateMachine.add('SEGMENT', sm_con, transitions={
                                   'succeeded': 'FEATURE_EXTRACTION'})
            smach.StateMachine.add('FEATURE_EXTRACTION', self.FeatureExtraction(), transitions={
                                   'succeeded': 'succeeded'})

    class SegmentYolo(smach.State):
        '''
        Segment using YOLO

        This should be turned into / merged with generic states
        '''

        def __init__(self):
            smach.State.__init__(self, outcomes=['succeeded', 'failed'], input_keys=[
                                 'img_msg_2d'], output_keys=['people_detections'])
            self.yolo = rospy.ServiceProxy('/yolov8/detect', YoloDetection)

        def execute(self, userdata):
            try:
                result = self.yolo(
                    userdata.img_msg_2d, "yolov8n-seg.pt", 0.5, 0.3)
                userdata.people_detections = [
                    det for det in result.detected_objects if det.name == "person"]
                return 'succeeded'
            except rospy.ServiceException as e:
                rospy.logwarn(f"Unable to perform inference. ({str(e)})")
                return 'failed'

    class SegmentBodypix(smach.State):
        '''
        Segment using Bodypix

        This should be turned into / merged with generic states
        '''

        def __init__(self):
            smach.State.__init__(self, outcomes=['succeeded', 'failed'], input_keys=[
                                 'img_msg_2d', 'xyz'], output_keys=['bodypix_masks'])
            self.bodypix = rospy.ServiceProxy(
                '/bodypix/detect', BodyPixDetection)

        def execute(self, userdata):
            try:
                torso = BodyPixMaskRequest()
                torso.parts = ["torso_front", "torso_back"]

                head = BodyPixMaskRequest()
                head.parts = ["left_face", "right_face"]

                masks = [torso, head]

                result = self.bodypix(userdata.img_msg_2d, "resnet50", 0.7, masks)
                userdata.bodypix_masks = result.masks
                rospy.loginfo("Found:::%s" % str(len(result.poses)))
                neck_coord = (int(result.poses[0].coord[0]), int(result.poses[0].coord[1]))
                rospy.loginfo("COORD_XY:::%s" % str(neck_coord))
                xyz = userdata.xyz
                xyz = np.nanmean(xyz, axis=2)
                rospy.loginfo("COORD_Z:::%s" % str(xyz[neck_coord[0]][neck_coord[1]]))
                return 'succeeded'
            except rospy.ServiceException as e:
                rospy.logwarn(f"Unable to perform inference. ({str(e)})")
                return 'failed'

    class FeatureExtraction(smach.State):
        '''
        Perform feature extraction

        This could be split into more states in theory, but that might just be unnecessary work
        '''

        def __init__(self):
            smach.State.__init__(self, outcomes=['succeeded', 'failed'], input_keys=[
                                 'img', 'people_detections', 'bodypix_masks'], output_keys=['people'])
            self.torch_face_features = rospy.ServiceProxy(
                '/torch/detect/face_features', TorchFaceFeatureDetection)

        def execute(self, userdata):
            if len(userdata.people_detections) == 0:
                rospy.logerr("Couldn't find anyone!")
                return 'failed'
            elif len(userdata.people_detections) == 1:
                rospy.logdebug("There is one person.")
            else:
                rospy.logdebug(
                    f"There are {len(userdata.people_detections)} people.")

            img = userdata.img
            height, width, _ = img.shape

            people = []

            for person in userdata.people_detections:
                rospy.logdebug(
                    f"\n\nFound person with confidence {person.confidence}!")

                # mask for this person
                mask_image = np.zeros((height, width), np.uint8)
                contours = np.array(person.xyseg).reshape(-1, 2)
                cv2.fillPoly(mask_image, pts=np.int32(
                    [contours]), color=(255, 255, 255))
                mask_bin = mask_image > 128

                # keep track
                features = []

                # process part masks
                for (bodypix_mask, part) in zip(userdata.bodypix_masks, ['torso', 'head']):
                    part_mask = np.array(bodypix_mask.mask).reshape(
                        bodypix_mask.shape[0], bodypix_mask.shape[1])
                    
                    # filter out part for current person segmentation
                    try:
                        part_mask[mask_bin == 0] = 0
                    except Exception:
                        rospy.logdebug('|> Failed to check {part} is visible')
                        continue

                    if part_mask.any():
                        rospy.logdebug(f'|> Person has {part} visible')
                    else:
                        rospy.logdebug(
                            f'|> Person does not have {part} visible')
                        continue

                    if part == 'torso':
                        torso_mask = part_mask
                    elif part == 'head':
                        head_mask = part_mask

                torso_mask_data, torso_mask_shape, torso_mask_dtype = numpy2message(torso_mask)
                head_mask_data, head_mask_shape, head_mask_dtype = numpy2message(head_mask)

                full_frame = cv2_img.cv2_img_to_msg(img)
                features.extend(self.torch_face_features(
                    full_frame, 
                    head_mask_data, head_mask_shape, head_mask_dtype,
                    torso_mask_data, torso_mask_shape, torso_mask_dtype,
                ).detected_features)

                # # process part masks
                # for (bodypix_mask, part) in zip(userdata.bodypix_masks, ['torso', 'head']):
                #     part_mask = np.array(bodypix_mask.mask).reshape(
                #         bodypix_mask.shape[0], bodypix_mask.shape[1])

                #     # filter out part for current person segmentation
                #     try:
                #         part_mask[mask_bin == 0] = 0
                #     except Exception:
                #         rospy.logdebug('|> Failed to check {part} is visible')
                #         continue

                #     if part_mask.any():
                #         rospy.logdebug(f'|> Person has {part} visible')
                #     else:
                #         rospy.logdebug(
                #             f'|> Person does not have {part} visible')
                #         continue

                #     # do colour processing on the torso
                #     if part == 'torso':
                #         try:
                #             features.append(FeatureWithColour("torso", [
                #                 ColourPrediction(colour, distance)
                #                 for colour, distance
                #                 in closest_colours(np.median(img[part_mask == 1], axis=0), RGB_COLOURS)
                #             ]))
                #         except Exception as e:
                #             rospy.logerr(f"Failed to process colour: {e}")

                #     # do feature extraction on the head
                #     if part == 'head':
                #         try:
                #             # crop out face
                #             face_mask = np.array(userdata.bodypix_masks[1].mask).reshape(
                #                 userdata.bodypix_masks[1].shape[0], userdata.bodypix_masks[1].shape[1])

                #             mask_image_only_face = mask_image.copy()
                #             mask_image_only_face[face_mask == 0] = 0

                #             face_region = cv2_img.extract_mask_region(
                #                 img, mask_image_only_face)
                #             if face_region is None:
                #                 raise Exception(
                #                     "Failed to extract mask region")

                #             msg = cv2_img.cv2_img_to_msg(face_region)
                #             features.extend(self.torch_face_features(
                #                 msg, False).detected_features)
                #         except Exception as e:
                #             rospy.logerr(f"Failed to process extraction: {e}")

                people.append({
                    'detection': person,
                    'features': features
                })

            # Userdata:
            # - people
            #   - - detection (YOLO)
            #     - parts
            #       - - part
            #         - mask

            userdata['people'] = people
            return 'succeeded'
