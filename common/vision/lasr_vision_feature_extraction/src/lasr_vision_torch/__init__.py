from feature_extractor.modules import UNetWithResnetEncoder, MultiLabelResNet, CombinedModel
from feature_extractor.helpers import load_torch_model, binary_erosion_dilation
from lasr_vision_feature_extraction.categories_and_attributes import CategoriesAndAttributes, CelebAMaskHQCategoriesAndAttributes
from lasr_vision_feature_extraction.image_with_masks_and_attributes import ImageWithMasksAndAttributes, ImageOfPerson

import numpy as np
import cv2
import torch
import rospkg
from os import path


class Predictor:
    def __init__(self, model: torch.nn.Module, device: torch.device, categories_and_attributes: CategoriesAndAttributes):
        self.model = model
        self.device = device
        self.categories_and_attributes = categories_and_attributes

        self._thresholds_mask: list[float] = []
        self._thresholds_pred: list[float] = []
        for key in sorted(list(self.categories_and_attributes.merged_categories.keys())):
            self._thresholds_mask.append(self.categories_and_attributes.thresholds_mask[key])
        for attribute in self.categories_and_attributes.attributes:
            if attribute not in self.categories_and_attributes.avoided_attributes:
                self._thresholds_pred.append(self.categories_and_attributes.thresholds_pred[attribute])

    def predict(self, rgb_image: np.ndarray) -> ImageWithMasksAndAttributes:
        mean_val = np.mean(rgb_image)
        image_tensor = torch.from_numpy(rgb_image).permute(2, 0, 1).unsqueeze(0).float() / 255.0
        pred_masks, pred_classes = self.model(image_tensor)
        # Apply binary erosion and dilation to the masks
        pred_masks = binary_erosion_dilation(
            pred_masks, thresholds=self._thresholds_mask,
            erosion_iterations=1, dilation_iterations=1
        )
        pred_masks = pred_masks.detach().squeeze(0).numpy().astype(np.uint8)
        mask_list = [pred_masks[i, :, :] for i in range(pred_masks.shape[0])]
        pred_classes = pred_classes.detach().squeeze(0).numpy()
        class_list = [pred_classes[i].item() for i in range(pred_classes.shape[0])]
        # print(rgb_image)
        print(mean_val)
        print(pred_classes)
        mask_dict = {}
        for i, mask in enumerate(mask_list):
            mask_dict[self.categories_and_attributes.mask_categories[i]] = mask
        attribute_dict = {}
        class_list_iter = class_list.__iter__()
        for attribute in self.categories_and_attributes.attributes:
            if attribute not in self.categories_and_attributes.avoided_attributes:
                attribute_dict[attribute] = class_list_iter.__next__()
        for attribute in self.categories_and_attributes.mask_labels:
            attribute_dict[attribute] = class_list_iter.__next__()
        image_obj = ImageWithMasksAndAttributes(rgb_image, mask_dict, attribute_dict, self.categories_and_attributes)
        return image_obj


def load_face_classifier_model():
    cat_layers = CelebAMaskHQCategoriesAndAttributes.merged_categories.keys().__len__()
    segment_model = UNetWithResnetEncoder(num_classes=cat_layers)
    predictions = len(CelebAMaskHQCategoriesAndAttributes.attributes) - len(
        CelebAMaskHQCategoriesAndAttributes.avoided_attributes) + len(CelebAMaskHQCategoriesAndAttributes.mask_labels)
    predict_model = MultiLabelResNet(num_labels=predictions, input_channels=cat_layers + 3)
    model = CombinedModel(segment_model, predict_model, cat_layers=cat_layers)
    model.eval()

    r = rospkg.RosPack()
    model, _, _, _ = load_torch_model(model, None, path=path.join(r.get_path(
        "lasr_vision_feature_extraction"), "models", "model.pth"), cpu_only=True)
    return model


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


def predict_frame(head_frame, torso_frame, full_frame, head_mask, torso_mask, predictor):
    full_frame = cv2.cvtColor(full_frame, cv2.COLOR_BGR2RGB)
    head_frame = cv2.cvtColor(head_frame, cv2.COLOR_BGR2RGB)
    torso_frame = cv2.cvtColor(torso_frame, cv2.COLOR_BGR2RGB)

    head_frame = pad_image_to_even_dims(head_frame)
    torso_frame = pad_image_to_even_dims(torso_frame)

    rst = ImageOfPerson.from_parent_instance(predictor.predict(head_frame))

    return rst.describe()
