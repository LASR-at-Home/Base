import torch
import torch.nn as nn
import torch.nn.functional as F
import torchvision.models as models


class CombinedModelNoRegression(nn.Module):
    def __init__(self, segment_model: nn.Module, predict_model: nn.Module, cat_layers: int = None):
        super(CombinedModelNoRegression, self).__init__()
        self.segment_model = segment_model
        self.predict_model = predict_model
        self.cat_layers = cat_layers

    def forward(self, x: torch.Tensor):
        seg_masks = self.segment_model(x)
        if self.cat_layers:
            seg_masks_ = seg_masks[:, 0:self.cat_layers]
            x = torch.cat((x, seg_masks_), dim=1)
        else:
            x = torch.cat((x, seg_masks), dim=1)
        logic_outputs = self.predict_model(x)
        return seg_masks, logic_outputs


class ASPP(nn.Module):
    def __init__(self, in_channels, out_channels):
        super(ASPP, self).__init__()
        self.atrous_block1 = nn.Conv2d(in_channels, out_channels, 1, 1)
        self.atrous_block6 = nn.Conv2d(
            in_channels, out_channels, 3, padding=6, dilation=6)
        self.atrous_block12 = nn.Conv2d(
            in_channels, out_channels, 3, padding=12, dilation=12)
        self.atrous_block18 = nn.Conv2d(
            in_channels, out_channels, 3, padding=18, dilation=18)
        self.conv_out = nn.Conv2d(out_channels * 4, out_channels, 1, 1)

    def forward(self, x):
        x1 = self.atrous_block1(x)
        x6 = self.atrous_block6(x)
        x12 = self.atrous_block12(x)
        x18 = self.atrous_block18(x)
        x = torch.cat([x1, x6, x12, x18], dim=1)
        return self.conv_out(x)


class DeepLabV3PlusMobileNetV3(nn.Module):
    def __init__(self, num_classes, in_channels=3, sigmoid=True):
        super(DeepLabV3PlusMobileNetV3, self).__init__()
        self.sigmoid = sigmoid
        mobilenet_v3 = models.mobilenet_v3_large(pretrained=True)

        if in_channels != 3:
            mobilenet_v3.features[0][0] = nn.Conv2d(
                in_channels, 16, kernel_size=3, stride=2, padding=1, bias=False
            )

        self.encoder = mobilenet_v3.features

        intermediate_channel = self.encoder[-1].out_channels
        self.aspp = ASPP(intermediate_channel, 256)

        self.decoder = nn.Sequential(
            # Concatenated with original input
            nn.Conv2d(256 + in_channels, 256, kernel_size=3, padding=1),
            nn.ReLU(inplace=True),
            nn.Conv2d(256, 256, kernel_size=3, padding=1),
            nn.ReLU(inplace=True),
            nn.Conv2d(256, num_classes, kernel_size=1)
        )

    def forward(self, x):
        original_input = x
        x_encoded = self.encoder(x)
        x_aspp = self.aspp(x_encoded)

        x = F.interpolate(
            x_aspp, size=original_input.shape[2:], mode='bilinear', align_corners=False)
        # Concatenate with original input
        x = torch.cat([x, original_input], dim=1)
        x = self.decoder(x)

        if self.sigmoid:
            x = torch.sigmoid(x)

        return x


class MultiLabelMobileNetV3Small(nn.Module):
    def __init__(self, num_labels, input_channels=3, sigmoid=True, pretrained=True):
        super(MultiLabelMobileNetV3Small, self).__init__()
        mobilenet_v3_small = models.mobilenet_v3_small(pretrained=pretrained)
        self.sigmoid = sigmoid

        if input_channels != 3:
            mobilenet_v3_small.features[0][0] = nn.Conv2d(
                input_channels, 16, kernel_size=3, stride=2, padding=1, bias=False
            )

        self.model = mobilenet_v3_small

        num_ftrs = self.model.classifier[3].in_features
        self.model.classifier[3] = nn.Linear(num_ftrs, num_labels)

    def forward(self, x):
        x = self.model(x)
        if self.sigmoid:
            x = torch.sigmoid(x)
        return x


class MultiLabelMobileNetV3Large(nn.Module):
    def __init__(self, num_labels, input_channels=3, sigmoid=True, pretrained=True):
        super(MultiLabelMobileNetV3Large, self).__init__()
        mobilenet_v3_small = models.mobilenet_v3_large(pretrained=pretrained)
        self.sigmoid = sigmoid

        if input_channels != 3:
            mobilenet_v3_small.features[0][0] = nn.Conv2d(
                input_channels, 16, kernel_size=3, stride=2, padding=1, bias=False
            )

        self.model = mobilenet_v3_small

        num_ftrs = self.model.classifier[3].in_features
        self.model.classifier[3] = nn.Linear(num_ftrs, num_labels)

    def forward(self, x):
        x = self.model(x)
        if self.sigmoid:
            x = torch.sigmoid(x)
        return x
