from torch_module.modules import UNetWithResnet18Encoder, MultiLabelResNet, CombinedModel  # DeepLabV3PlusMobileNetV3, MultiLabelMobileNetV3Large, CombinedModelNoRegression
from torch_module.helpers import load_torch_model, binary_erosion_dilation

from colour_estimation import load_images_to_dict, generate_colour_table, count_colours_in_masked_area, compare_colour_distributions
from colour_estimation import SPESIFIC_COLOURS, DETAILED_COLOURS

import numpy as np
import cv2
import torch
import rospy
import rospkg
import lasr_vision_torch
from os import path
# import matplotlib.pyplot as plt


def load_face_classifier_model():
    cat_layers = 4
    segment_model = UNetWithResnet18Encoder(num_classes=4)
    predict_model = MultiLabelResNet(num_labels=4, input_channels=7)
    model = CombinedModel(segment_model, predict_model, cat_layers=cat_layers)
    model.eval()

    r = rospkg.RosPack()
    model, _, _, _ = load_torch_model(model, None, path=path.join(r.get_path(
        "lasr_vision_torch"), "models", "model.pth"), cpu_only=True)
    return model


model = load_face_classifier_model()
# setups
face_th_rate = 0.05
thresholds_mask = [
    0.5, 0.75, 0.25, 0.5,  # 0.5, 0.5, 0.5, 0.5,
]
thresholds_pred = [
    0.6, 0.8, 0.05, 0.5,
]
erosion_iterations = 1
dilation_iterations = 1
colour_distance_rate = 1.2
categories = ['hair', 'hat', 'glasses', 'face',]
cat_layers = 4

# prepare hair colour table
r = rospkg.RosPack()
image_dict = load_images_to_dict(path.join(r.get_path(
        "colour_estimation"), "hair_colours"))
hair_colour_table = generate_colour_table(image_dict, SPESIFIC_COLOURS)


def pad_image_to_even_dims(image):
    # Get the current shape of the image
    height, width, _ = image.shape

    # Calculate the padding needed for height and width
    height_pad = 0 if height % 2 == 0 else 1
    width_pad = 0 if width % 2 == 0 else 1

    # Pad the image. Pad the bottom and right side of the image
    padded_image = np.pad(image, ((0, height_pad), (0, width_pad), (0, 0)), mode='constant', constant_values=0)

    return padded_image


def extract_mask_region(frame, mask, expand_x=0.5, expand_y=0.5):
    """
    Extracts the face region from the image and expands the region by the specified amount.
    
    :param frame: The source image.
    :param mask: The mask with the face part.
    :param expand_x: The percentage to expand the width of the bounding box.
    :param expand_y: The percentage to expand the height of the bounding box.
    :return: The extracted face region as a numpy array, or None if not found.
    """
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
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


def process_head(head_frame, model, thresholds_mask, erosion_iterations, dilation_iterations, thresholds_pred):
    """
    Processes the head frame to extract class counts and color information for head-related classes.

    Args:
    - head_frame (np.ndarray): The head frame extracted by the BodyPix model.
    - model: A PyTorch model instance for classifying and predicting masks for head features.
    - thresholds_mask, erosion_iterations, dilation_iterations: Thresholds and iteration counts for binary erosion and dilation.
    - thresholds_pred: A list of prediction thresholds.

    Returns:
    - Tuple[dict, dict]: A tuple containing two dictionaries:
        - head_class_count: A dictionary with counts for each head-related class.
        - head_class_colours: A dictionary with color information for each head-related class.
    """
    head_class_count = {
        'hair': 0,
        'hat': 0,
        'glasses': 0,
    }
    head_class_colours = {
        'hair': {},
        'hat': {},
        'glasses': {},
    }

    if head_frame is not None:
        # try:
        #     r = rospkg.RosPack()
        #     _head_frame_bgr = cv2.cvtColor(head_frame, cv2.COLOR_RGB2BGR)
        #     cv2.imwrite(path.join(r.get_path("lasr_vision_torch"), 'head_frame.jpg'), _head_frame_bgr)
        # except Exception as ignore:
        #     pass

        # Convert head frame to PyTorch tensor and normalize
        head_frame_tensor = torch.from_numpy(head_frame).permute(2, 0, 1).unsqueeze(0).float() / 255.0
        masks_batch_pred, pred_classes = model(head_frame_tensor)

        # Apply binary erosion and dilation to the masks
        processed_masks = binary_erosion_dilation(
            masks_batch_pred, thresholds=thresholds_mask, 
            erosion_iterations=erosion_iterations, dilation_iterations=dilation_iterations
        )
        masks = processed_masks.detach().squeeze(0).numpy().astype(np.uint8)
        mask_list = [masks[i,:,:] for i in range(masks.shape[0])]
        pred_classes = pred_classes.detach().squeeze(0).numpy()

        # Determine if each class is present
        class_list = [pred_classes[i].item() > thresholds_pred[i] for i in range(pred_classes.shape[0])]

        # Update class count
        for each_class, k in zip(class_list[0:3], ['hair', 'hat', 'glasses']):
            head_class_count[k] = int(each_class)

        # Update class colours
        for f, each_mask, k, c_map in zip([head_frame, head_frame, head_frame], mask_list[0:2], ['hair', 'hat', 'glasses'], [SPESIFIC_COLOURS, DETAILED_COLOURS, DETAILED_COLOURS]):
            colours = count_colours_in_masked_area(f, each_mask, c_map, sort=True)[1]
                # colours = [c in ]
            for colour in colours:
                head_class_colours[k][colour[0]] = colour[1]
                # if colour[0] not in head_class_colours[k]:
                #     head_class_colours[k][colour[0]] = [colour[1]]
                # else:
                #     head_class_colours[k][colour[0]].append(colour[1])

    return head_class_count, head_class_colours


def process_cloth(full_frame, torso_mask):
    """
    Processes the full frame with the torso mask to extract class counts and color information for cloth.

    Args:
    - full_frame (np.ndarray): The full original frame from the video source.
    - torso_mask (np.ndarray): The torso mask extracted by the BodyPix model.

    Returns:
    - Tuple[dict, dict]: A tuple containing two dictionaries:
        - cloth_class_count: A dictionary with counts for the cloth class.
        - cloth_class_colours: A dictionary with color information for the cloth class.
    """
    cloth_class_count = {
        'cloth': 0,
    }
    cloth_class_colours = {
        'cloth': {},
    }

    # Check if cloth is detected
    if torso_mask is not None and np.sum(torso_mask) >= 50:
        cloth_class_count['cloth'] = 1

        # Update cloth colours
        colours = count_colours_in_masked_area(full_frame, torso_mask, DETAILED_COLOURS, sort=True)[1]
        for colour in colours:
            cloth_class_colours['cloth'][colour[0]] = colour[1]
            # if colour[0] not in cloth_class_colours['cloth']:
            #     cloth_class_colours['cloth'][colour[0]] = [colour[1]]
            # else:
            #     cloth_class_colours['cloth'][colour[0]].append(colour[1])

    return cloth_class_count, cloth_class_colours


# you can use this function directly for prediction.
def predict_frame(head_frame, torso_frame, full_frame, head_mask, torso_mask, model, thresholds_mask, erosion_iterations, dilation_iterations, thresholds_pred):
    """
    Predicts classes and color information for a single processed video frame.

    Args:
    - head_frame (np.ndarray): The head frame extracted by the BodyPix model.
    - full_frame (np.ndarray): The full original frame from the video source.
    - head_mask (np.ndarray): The head mask extracted by the BodyPix model.
    - torso_mask (np.ndarray): The torso mask extracted by the BodyPix model.
    - model: A PyTorch model instance for classifying and predicting masks for head features.
    - thresholds_mask, erosion_iterations, dilation_iterations: Thresholds and iteration counts for binary erosion and dilation.
    - thresholds_pred: A list of prediction thresholds.

    Returns:
    - Tuple[dict, dict]: A tuple containing:
        - class_pred: A dictionary with predicted classes for the single frame.
        - colour_pred: A dictionary with predicted colors for the single frame.
    """
    class_count = {
        'hair': 0,
        'hat': 0,
        'glasses': 0,
        'cloth': 0,
    }
    class_colours = {
        'hair': {},
        'hat': {},
        'glasses': {},
        'cloth': {},
    }

    full_frame = cv2.cvtColor(full_frame, cv2.COLOR_BGR2RGB)
    head_frame = cv2.cvtColor(head_frame, cv2.COLOR_BGR2RGB)
    torso_frame = cv2.cvtColor(torso_frame, cv2.COLOR_BGR2RGB)

    head_frame = pad_image_to_even_dims(head_frame)
    torso_frame = pad_image_to_even_dims(torso_frame)
    
    # cv2 imshow is currently not working, not knowing why...
    # try:
    #     r = rospkg.RosPack()
    #     _full_frame_bgr = cv2.cvtColor(full_frame, cv2.COLOR_RGB2BGR)
    #     cv2.imwrite(path.join(r.get_path("lasr_vision_torch"), 'full_frame.jpg'), _full_frame_bgr)
    #     _head_frame_bgr = cv2.cvtColor(head_frame, cv2.COLOR_RGB2BGR)
    #     cv2.imwrite(path.join(r.get_path("lasr_vision_torch"), 'head_frame.jpg'), _head_frame_bgr)
    #     _torso_frame_bgr = cv2.cvtColor(torso_frame, cv2.COLOR_RGB2BGR)
    #     cv2.imwrite(path.join(r.get_path("lasr_vision_torch"), 'torso_frame.jpg'), _torso_frame_bgr)
    # except Exception as ignore:
    #     pass

    # Process head and cloth separately for the single frame
    head_class_count, head_class_colours = process_head(head_frame, model, thresholds_mask, erosion_iterations, dilation_iterations, thresholds_pred)
    cloth_class_count, cloth_class_colours = process_cloth(full_frame, torso_mask)

    # Update class counts and colours
    for k in head_class_count:
        class_count[k] = head_class_count[k]
        class_colours[k] = head_class_colours[k]

    class_count['cloth'] = cloth_class_count['cloth']
    class_colours['cloth'] = cloth_class_colours['cloth']

    # Compute final class predictions and colors for the single frame
    class_pred = {k: bool(class_count[k]) for k in class_count}
    colour_pred = {k: v for k, v in class_colours.items()}

    rospy.loginfo(str(class_colours['hair']))
    rospy.loginfo(str(hair_colour_table))

    # compare_colour_distributions([k,v class_colours['hair']], hair_colour_table)
    colour_pred['hair'] = compare_colour_distributions(class_colours['hair'], hair_colour_table)

    # class_pred, colour_pred = None, None

    return class_pred, colour_pred


# # if able to provide multiple frames (see __main__ seciton), then this should work better than the single frame version.
# def predict_frames(head_frames, torso_frames, full_frames, head_masks, torso_masks, model, thresholds_mask, erosion_iterations, dilation_iterations, thresholds_pred, SPESIFIC_COLOURS):
#     """
#     Predicts classes and color information for a sequence of processed video frames.

#     Args:
#     - head_frames (list[np.ndarray]): List of head frames extracted by the BodyPix model.
#     - torso_frames (list[np.ndarray]): List of body frames extracted by the BodyPix model.
#     - full_frames (list[np.ndarray]): List of full original frames from the video source.
#     - head_masks (list[np.ndarray]): List of head masks extracted by the BodyPix model.
#     - torso_masks (list[np.ndarray]): List of torso masks extracted by the BodyPix model.
#     - model: A PyTorch model instance for classifying and predicting masks for head features.
#     - thresholds_mask, erosion_iterations, dilation_iterations: Thresholds and iteration counts for binary erosion and dilation.
#     - thresholds_pred: A list of prediction thresholds.
#     - SPESIFIC_COLOURS: A dictionary of specific colors.

#     Returns:
#     - Tuple[dict, dict]: A tuple containing:
#         - class_pred: A dictionary with predicted classes.
#         - colour_pred: A dictionary with predicted colors.
#     """
#     total_class_count = {
#         'hair': [],
#         'hat': [],
#         'glasses': [],
#         'cloth': [],
#     }
#     total_class_colours = {
#         'hair': {},
#         'hat': {},
#         'glasses': {},
#         'cloth': {},
#     }

#     for head_frame, torso_frame, full_frame, head_mask, torso_mask in zip(head_frames, torso_frames, full_frames, head_masks, torso_masks):
#         head_frame = pad_image_to_even_dims(head_frame)
#         torso_frame = pad_image_to_even_dims(torso_frame)

#         # Process head and cloth separately
#         head_class_count, head_class_colours = process_head(head_frame, model, thresholds_mask, erosion_iterations, dilation_iterations, thresholds_pred)
#         cloth_class_count, cloth_class_colours = process_cloth(full_frame, torso_mask)

#         # Accumulate class counts and colours
#         for k in head_class_count:
#             total_class_count[k].append(head_class_count[k])
#             if k in head_class_colours:
#                 for colour, count in head_class_colours[k].items():
#                     if colour not in total_class_colours[k]:
#                         total_class_colours[k][colour] = count
#                     else:
#                         total_class_colours[k][colour].extend(count)

#         total_class_count['cloth'].append(cloth_class_count['cloth'])
#         for colour, count in cloth_class_colours['cloth'].items():
#             if colour not in total_class_colours['cloth']:
#                 total_class_colours['cloth'][colour] = count
#             else:
#                 total_class_colours['cloth'][colour].extend(count)

#     # Compute final class predictions and colors
#     class_pred = {k: sum(v) >= len(v) / 2 for k, v in total_class_count.items()}
#     colour_pred = average_colours_by_label(total_class_count, total_class_colours)

#     return class_pred, colour_pred

