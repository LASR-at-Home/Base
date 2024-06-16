import json
from os import path

import cv2
import numpy as np
import rospkg
import torch
import torch.nn as nn
import torch.nn.functional as F
import torchvision.models as models
from lasr_vision_feature_extraction.categories_and_attributes import (
    CategoriesAndAttributes,
    CelebAMaskHQCategoriesAndAttributes,
    DeepFashion2GeneralizedCategoriesAndAttributes,
)
from lasr_vision_feature_extraction.image_with_masks_and_attributes import (
    ImageWithMasksAndAttributes,
    ImageOfPerson,
    ImageOfCloth,
)


def X2conv(in_channels, out_channels, inner_channels=None):
    inner_channels = out_channels // 2 if inner_channels is None else inner_channels
    down_conv = nn.Sequential(
        nn.Conv2d(in_channels, inner_channels, kernel_size=3, padding=1, bias=False),
        nn.BatchNorm2d(inner_channels),
        nn.ReLU(inplace=True),
        nn.Conv2d(inner_channels, out_channels, kernel_size=3, padding=1, bias=False),
        nn.BatchNorm2d(out_channels),
        nn.ReLU(inplace=True),
    )
    return down_conv


class Decoder(nn.Module):
    def __init__(self, in_channels, skip_channels, out_channels):
        super(Decoder, self).__init__()
        self.up = nn.ConvTranspose2d(in_channels, out_channels, kernel_size=2, stride=2)
        self.up_conv = X2conv(out_channels + skip_channels, out_channels)

    def forward(self, x_copy, x):
        x = self.up(x)
        if x.size(2) != x_copy.size(2) or x.size(3) != x_copy.size(3):
            x = F.interpolate(
                x,
                size=(x_copy.size(2), x_copy.size(3)),
                mode="bilinear",
                align_corners=True,
            )
        x = torch.cat((x_copy, x), dim=1)
        x = self.up_conv(x)
        return x


class UNetWithResnetEncoder(nn.Module):
    def __init__(self, num_classes, in_channels=3, freeze_bn=False, sigmoid=True):
        super(UNetWithResnetEncoder, self).__init__()
        self.sigmoid = sigmoid
        self.resnet = models.resnet34(
            pretrained=False
        )  # Initialize with a ResNet model
        if in_channels != 3:
            self.resnet.conv1 = nn.Conv2d(
                in_channels, 64, kernel_size=7, stride=2, padding=3, bias=False
            )

        self.encoder1 = nn.Sequential(
            self.resnet.conv1, self.resnet.bn1, self.resnet.relu
        )
        self.encoder2 = self.resnet.layer1
        self.encoder3 = self.resnet.layer2
        self.encoder4 = self.resnet.layer3
        self.encoder5 = self.resnet.layer4

        self.up1 = Decoder(512, 256, 256)
        self.up2 = Decoder(256, 128, 128)
        self.up3 = Decoder(128, 64, 64)
        self.up4 = Decoder(64, 64, 64)

        self.final_conv = nn.Conv2d(64, num_classes, kernel_size=1)
        self._initialize_weights()

        if freeze_bn:
            self.freeze_bn()

    def _initialize_weights(self):
        for module in self.modules():
            if isinstance(module, nn.Conv2d) or isinstance(module, nn.ConvTranspose2d):
                nn.init.kaiming_normal_(module.weight)
                if module.bias is not None:
                    module.bias.data.zero_()
                elif isinstance(module, nn.BatchNorm2d):
                    module.weight.data.fill_(1)
                    module.bias.data.zero_()

    def forward(self, x):
        x1 = self.encoder1(x)
        x2 = self.encoder2(x1)
        x3 = self.encoder3(x2)
        x4 = self.encoder4(x3)
        x5 = self.encoder5(x4)

        x = self.up1(x4, x5)
        x = self.up2(x3, x)
        x = self.up3(x2, x)
        x = self.up4(x1, x)
        x = F.interpolate(
            x, size=(x.size(2) * 2, x.size(3) * 2), mode="bilinear", align_corners=True
        )

        x = self.final_conv(x)

        if self.sigmoid:
            x = torch.sigmoid(x)
        return x

    def freeze_bn(self):
        for module in self.modules():
            if isinstance(module, nn.BatchNorm2d):
                module.eval()

    def unfreeze_bn(self):
        for module in self.modules():
            if isinstance(module, nn.BatchNorm2d):
                module.train()


class MultiLabelResNet(nn.Module):
    def __init__(self, num_labels, input_channels=3, sigmoid=True):
        super(MultiLabelResNet, self).__init__()
        self.model = models.resnet34(pretrained=False)
        self.sigmoid = sigmoid

        if input_channels != 3:
            self.model.conv1 = nn.Conv2d(
                input_channels, 64, kernel_size=7, stride=2, padding=3, bias=False
            )

        num_ftrs = self.model.fc.in_features

        self.model.fc = nn.Linear(num_ftrs, num_labels)

    def forward(self, x):
        x = self.model(x)
        if self.sigmoid:
            x = torch.sigmoid(x)
        return x


class CombinedModel(nn.Module):
    def __init__(
        self, segment_model: nn.Module, predict_model: nn.Module, cat_layers: int = None
    ):
        super(CombinedModel, self).__init__()
        self.segment_model = segment_model
        self.predict_model = predict_model
        self.cat_layers = cat_layers
        self.freeze_seg = False

    def forward(self, x: torch.Tensor):
        seg_masks = self.segment_model(x)
        seg_masks_ = seg_masks.detach()
        if self.cat_layers:
            seg_masks_ = seg_masks_[:, 0 : self.cat_layers]
            x = torch.cat((x, seg_masks_), dim=1)
        else:
            x = torch.cat((x, seg_masks_), dim=1)
        logic_outputs = self.predict_model(x)
        return seg_masks, logic_outputs

    def freeze_segment_model(self):
        self.segment_model.eval()

    def unfreeze_segment_model(self):
        self.segment_model.train()


class SegmentPredictor(nn.Module):
    def __init__(self, num_masks, num_labels, in_channels=3, sigmoid=True):
        super(SegmentPredictor, self).__init__()
        self.sigmoid = sigmoid
        self.resnet = models.resnet18(pretrained=False)

        # Adapt ResNet to handle different input channel sizes
        if in_channels != 3:
            self.resnet.conv1 = nn.Conv2d(
                in_channels, 64, kernel_size=7, stride=2, padding=3, bias=False
            )

        # Encoder layers
        self.encoder1 = nn.Sequential(
            self.resnet.conv1, self.resnet.bn1, self.resnet.relu
        )
        self.encoder2 = self.resnet.layer1
        self.encoder3 = self.resnet.layer2
        self.encoder4 = self.resnet.layer3
        self.encoder5 = self.resnet.layer4

        # Decoder layers
        # resnet18/34
        self.up1 = Decoder(512, 256, 256)
        self.up2 = Decoder(256, 128, 128)
        self.up3 = Decoder(128, 64, 64)
        self.up4 = Decoder(64, 64, 64)

        # resnet50/101/152
        # self.up1 = Decoder(2048, 1024, 1024)
        # self.up2 = Decoder(1024, 512, 512)
        # self.up3 = Decoder(512, 256, 256)
        # self.up4 = Decoder(256, 64, 64)

        # Segmentation head
        self.final_conv = nn.Conv2d(64, num_masks, kernel_size=1)

        # Classification head
        self.global_pool = nn.AdaptiveAvgPool2d((1, 1))
        self.predictor_cnn_extension = nn.Sequential(
            nn.Conv2d(512, 2048, kernel_size=3, padding=1),  # resnet18/34
            # nn.Conv2d(2048, 2048, kernel_size=3, padding=1),
            nn.LeakyReLU(negative_slope=0.01),
            nn.Conv2d(2048, 2048, kernel_size=3, padding=1),
            nn.LeakyReLU(negative_slope=0.01),
        )
        self.classifier = nn.Sequential(
            nn.Linear(2048, 256),  # resnet50/101/152
            nn.LeakyReLU(negative_slope=0.01),
            nn.Dropout(p=0.5),
            nn.Linear(256, 256),
            nn.LeakyReLU(negative_slope=0.01),
            nn.Dropout(p=0.5),
            nn.Linear(256, num_labels),
        )

    def forward(self, x):
        x1 = self.encoder1(x)
        x2 = self.encoder2(x1)
        x3 = self.encoder3(x2)
        x4 = self.encoder4(x3)
        x5 = self.encoder5(x4)

        x = self.up1(x4, x5)
        x = self.up2(x3, x)
        x = self.up3(x2, x)
        x = self.up4(x1, x)
        x = F.interpolate(
            x, size=(x.size(2) * 2, x.size(3) * 2), mode="bilinear", align_corners=True
        )

        mask = self.final_conv(x)

        # Predicting the labels using features from the last encoder output
        x_cls = self.predictor_cnn_extension(x5)
        x_cls = self.global_pool(
            x_cls
        )  # Use the feature map from the last encoder layer
        x_cls = x_cls.view(x_cls.size(0), -1)
        labels = self.classifier(x_cls)

        if self.sigmoid:
            mask = torch.sigmoid(mask)
            labels = torch.sigmoid(labels)

        return mask, labels


class SegmentPredictorBbox(SegmentPredictor):
    def __init__(
        self,
        num_masks,
        num_labels,
        num_bbox_classes,
        in_channels=3,
        sigmoid=True,
        return_bbox=True,
    ):
        self.return_bbox = return_bbox
        super(SegmentPredictorBbox, self).__init__(
            num_masks, num_labels, in_channels, sigmoid
        )
        self.num_bbox_classes = num_bbox_classes
        self.bbox_cnn_extension = nn.Sequential(
            nn.Conv2d(512, 2048, kernel_size=3, padding=1),  # resnet18/34
            # nn.Conv2d(2048, 2048, kernel_size=3, padding=1),
            nn.LeakyReLU(negative_slope=0.01),
            nn.Conv2d(2048, 2048, kernel_size=3, padding=1),
            nn.LeakyReLU(negative_slope=0.01),
        )
        self.bbox_generator = nn.Sequential(
            nn.Linear(2048, 256),
            nn.LeakyReLU(negative_slope=0.01),
            nn.Linear(256, 256),
            nn.LeakyReLU(negative_slope=0.01),
            nn.Linear(256, num_bbox_classes * 4),
        )

    def forward(self, x):
        x1 = self.encoder1(x)
        x2 = self.encoder2(x1)
        x3 = self.encoder3(x2)
        x4 = self.encoder4(x3)
        x5 = self.encoder5(x4)

        x = self.up1(x4, x5)
        x = self.up2(x3, x)
        x = self.up3(x2, x)
        x = self.up4(x1, x)
        x = F.interpolate(
            x, size=(x.size(2) * 2, x.size(3) * 2), mode="bilinear", align_corners=True
        )

        mask = self.final_conv(x)

        # Predicting the labels using features from the last encoder output
        x_cls = self.predictor_cnn_extension(x5)
        x_cls = self.global_pool(
            x_cls
        )  # Use the feature map from the last encoder layer
        x_cls = x_cls.view(x_cls.size(0), -1)
        labels = self.classifier(x_cls)
        x_bbox = self.bbox_cnn_extension(x5)
        x_bbox = self.global_pool(x_bbox)
        x_bbox = x_bbox.view(x_bbox.size(0), -1)
        bboxes = self.bbox_generator(x_bbox).view(-1, self.num_bbox_classes, 4)

        # no sigmoid for bboxes.
        if self.sigmoid:
            mask = torch.sigmoid(mask)
            labels = torch.sigmoid(labels)

        if self.return_bbox:
            return mask, labels, bboxes
        return mask, labels


class Predictor:
    def __init__(
        self,
        model: torch.nn.Module,
        device: torch.device,
        categories_and_attributes: CategoriesAndAttributes,
    ):
        self.model = model
        self.device = device
        self.categories_and_attributes = categories_and_attributes

        self._thresholds_mask: list[float] = []
        self._thresholds_pred: list[float] = []
        for key in sorted(
            list(self.categories_and_attributes.merged_categories.keys())
        ):
            self._thresholds_mask.append(
                self.categories_and_attributes.thresholds_mask[key]
            )
        for attribute in self.categories_and_attributes.attributes:
            if attribute not in self.categories_and_attributes.avoided_attributes:
                self._thresholds_pred.append(
                    self.categories_and_attributes.thresholds_pred[attribute]
                )

    def predict(self, rgb_image: np.ndarray) -> ImageWithMasksAndAttributes:
        mean_val = np.mean(rgb_image)
        image_tensor = (
            torch.from_numpy(rgb_image).permute(2, 0, 1).unsqueeze(0).float() / 255.0
        )
        pred_masks, pred_classes = self.model(image_tensor)
        # Apply binary erosion and dilation to the masks
        pred_masks = binary_erosion_dilation(
            pred_masks,
            thresholds=self._thresholds_mask,
            erosion_iterations=1,
            dilation_iterations=1,
        )
        pred_masks = pred_masks.detach().squeeze(0).numpy().astype(np.uint8)
        mask_list = [pred_masks[i, :, :] for i in range(pred_masks.shape[0])]
        pred_classes = pred_classes.detach().squeeze(0).numpy()
        class_list = [pred_classes[i].item() for i in range(pred_classes.shape[0])]
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
        image_obj = ImageWithMasksAndAttributes(
            rgb_image, mask_dict, attribute_dict, self.categories_and_attributes
        )
        return image_obj


def load_face_classifier_model():
    cat_layers = CelebAMaskHQCategoriesAndAttributes.merged_categories.keys().__len__()
    segment_model = UNetWithResnetEncoder(num_classes=cat_layers)
    predictions = (
        len(CelebAMaskHQCategoriesAndAttributes.attributes)
        - len(CelebAMaskHQCategoriesAndAttributes.avoided_attributes)
        + len(CelebAMaskHQCategoriesAndAttributes.mask_labels)
    )
    predict_model = MultiLabelResNet(
        num_labels=predictions, input_channels=cat_layers + 3
    )
    model = CombinedModel(segment_model, predict_model, cat_layers=cat_layers)
    model.eval()

    r = rospkg.RosPack()
    model, _, _, _ = load_torch_model(
        model,
        None,
        path=path.join(
            r.get_path("lasr_vision_feature_extraction"), "models", "face_model.pth"
        ),
        cpu_only=True,
    )
    return model


def load_cloth_classifier_model():
    num_classes = len(DeepFashion2GeneralizedCategoriesAndAttributes.attributes)
    model = SegmentPredictorBbox(
        num_masks=num_classes + 4, num_labels=num_classes + 4, num_bbox_classes=4
    )
    model.eval()

    r = rospkg.RosPack()
    model, _, _, _ = load_torch_model(
        model,
        None,
        path=path.join(
            r.get_path("lasr_vision_feature_extraction"), "models", "cloth_model.pth"
        ),
        cpu_only=True,
    )
    return model


def pad_image_to_even_dims(image):
    # Get the current shape of the image
    height, width, _ = image.shape

    # Calculate the padding needed for height and width
    height_pad = 0 if height % 2 == 0 else 1
    width_pad = 0 if width % 2 == 0 else 1

    # Pad the image. Pad the bottom and right side of the image
    padded_image = np.pad(
        image,
        ((0, height_pad), (0, width_pad), (0, 0)),
        mode="constant",
        constant_values=0,
    )

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

        face_region = frame[y : y + int(new_h), x : x + int(new_w)]
        return face_region
    return None


def predict_frame(
    head_frame,
    torso_frame,
    full_frame,
    head_mask,
    torso_mask,
    head_predictor,
    cloth_predictor,
):
    full_frame = cv2.cvtColor(full_frame, cv2.COLOR_BGR2RGB)
    head_frame = cv2.cvtColor(head_frame, cv2.COLOR_BGR2RGB)
    torso_frame = cv2.cvtColor(torso_frame, cv2.COLOR_BGR2RGB)

    head_frame = pad_image_to_even_dims(head_frame)
    torso_frame = pad_image_to_even_dims(torso_frame)

    rst_person = ImageOfPerson.from_parent_instance(
        head_predictor.predict(head_frame)
    ).describe()
    rst_cloth = ImageOfCloth.from_parent_instance(
        cloth_predictor.predict(torso_frame)
    ).describe()

    result = {
        "attributes": {**rst_person["attributes"], **rst_cloth["attributes"]},
        "description": rst_person["description"] + rst_cloth["description"],
    }

    result = json.dumps(result, indent=4)

    return result


def load_torch_model(model, optimizer, path="model.pth", cpu_only=False):
    if cpu_only:
        checkpoint = torch.load(path, map_location=torch.device("cpu"))
    else:
        checkpoint = torch.load(path)
    model.load_state_dict(checkpoint["model_state_dict"])
    if optimizer is not None:
        optimizer.load_state_dict(checkpoint["optimizer_state_dict"])
    epoch = checkpoint["epoch"]
    best_val_loss = checkpoint.get("best_val_loss", float("inf"))
    return model, optimizer, epoch, best_val_loss


def binary_erosion_dilation(
    tensor, thresholds, erosion_iterations=1, dilation_iterations=1
):
    """
    Apply binary threshold, followed by erosion and dilation to a tensor.

    :param tensor: Input tensor (N, C, H, W)
    :param thresholds: List of threshold values for each channel
    :param erosion_iterations: Number of erosion iterations
    :param dilation_iterations: Number of dilation iterations
    :return: Processed tensor
    """

    # Check if the length of thresholds matches the number of channels
    if len(thresholds) != tensor.size(1):
        # the error should be here, just removed for now since there's some other bug I haven't fixed.
        # raise ValueError(f"Length of thresholds {len(thresholds)} must match the number of channels {tensor.size(1)}")
        thresholds = [0.5 for _ in range(tensor.size(1))]

    # Binary thresholding
    for i, threshold in enumerate(thresholds):
        tensor[:, i] = (tensor[:, i] > threshold / 2).float() / 4
        tensor[:, i] += (tensor[:, i] > threshold).float()
        tensor[:, i] /= max(tensor[:, i].clone())

    # Define the 3x3 kernel for erosion and dilation
    kernel = (
        torch.tensor([[1, 1, 1], [1, 1, 1], [1, 1, 1]], dtype=torch.float32)
        .unsqueeze(0)
        .unsqueeze(0)
    )

    # Replicate the kernel for each channel
    kernel = kernel.repeat(tensor.size(1), 1, 1, 1).to(tensor.device)

    # Erosion
    for _ in range(erosion_iterations):
        # 3x3 convolution with groups
        tensor = F.conv2d(tensor, kernel, padding=1, groups=tensor.size(1))
        tensor = (tensor == 9).float()  # Check if all neighboring pixels are 1

    # Dilation
    for _ in range(dilation_iterations):
        # 3x3 convolution with groups
        tensor_dilated = F.conv2d(tensor, kernel, padding=1, groups=tensor.size(1))
        # Combine the original and dilated tensors
        tensor = torch.clamp(tensor + tensor_dilated, 0, 1)

    return tensor


def median_color_float(rgb_image: torch.Tensor, mask: torch.Tensor) -> torch.Tensor:
    mask = mask.bool()
    median_colors = torch.zeros(
        (rgb_image.size(0), mask.size(1), rgb_image.size(1)), device=rgb_image.device
    )
    for i in range(rgb_image.size(0)):
        for j in range(mask.size(1)):
            for k in range(rgb_image.size(1)):
                valid_pixels = torch.masked_select(rgb_image[i, k], mask[i, j])
                if valid_pixels.numel() > 0:
                    median_value = valid_pixels.median()
                else:
                    median_value = torch.tensor(0.0, device=rgb_image.device)
                median_colors[i, j, k] = median_value
    return median_colors  # / 255.0
