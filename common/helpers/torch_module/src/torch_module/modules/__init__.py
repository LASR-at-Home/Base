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
    

def x2conv(in_channels, out_channels, inner_channels=None):
    inner_channels = out_channels // 2 if inner_channels is None else inner_channels
    down_conv = nn.Sequential(
        nn.Conv2d(in_channels, inner_channels, kernel_size=3, padding=1, bias=False),
        nn.BatchNorm2d(inner_channels),
        nn.ReLU(inplace=True),
        nn.Conv2d(inner_channels, out_channels, kernel_size=3, padding=1, bias=False),
        nn.BatchNorm2d(out_channels),
        nn.ReLU(inplace=True))
    return down_conv


class Encoder(nn.Module):
    def __init__(self, in_channels, out_channels):
        super(Encoder, self).__init__()
        self.down_conv = x2conv(in_channels, out_channels)
        self.pool = nn.MaxPool2d(kernel_size=2, ceil_mode=True)

    def forward(self, x):
        x = self.down_conv(x)
        x = self.pool(x)
        return x


class Decoder(nn.Module):
    def __init__(self, in_channels, out_channels):
        super(Decoder, self).__init__()
        self.up = nn.ConvTranspose2d(in_channels, in_channels // 2, kernel_size=2, stride=2)
        self.up_conv = x2conv(in_channels, out_channels)

    def forward(self, x_copy, x, interpolate=True):
        x = self.up(x)

        if (x.size(2) != x_copy.size(2)) or (x.size(3) != x_copy.size(3)):
            if interpolate:
                # Iterpolating instead of padding
                x = F.interpolate(x, size=(x_copy.size(2), x_copy.size(3)),
                                mode="bilinear", align_corners=True)
            else:
                # Padding in case the incomping volumes are of different sizes
                diffY = x_copy.size()[2] - x.size()[2]
                diffX = x_copy.size()[3] - x.size()[3]
                x = F.pad(x, (diffX // 2, diffX - diffX // 2,
                                diffY // 2, diffY - diffY // 2))

        # Concatenate
        x = torch.cat([x_copy, x], dim=1)
        x = self.up_conv(x)
        return x


class UNetWithResnet18Encoder(nn.Module):
    class Decoder(nn.Module):
        def __init__(self, in_channels, skip_channels, out_channels):
            super(UNetWithResnet18Encoder.Decoder, self).__init__()
            self.up = nn.ConvTranspose2d(in_channels, out_channels, kernel_size=2, stride=2)
            self.up_conv = x2conv(out_channels + skip_channels, out_channels)

        def forward(self, x_copy, x):
            x = self.up(x)
            if x.size(2) != x_copy.size(2) or x.size(3) != x_copy.size(3):
                x = F.interpolate(x, size=(x_copy.size(2), x_copy.size(3)), mode='bilinear', align_corners=True)
            x = torch.cat((x_copy, x), dim=1)
            x = self.up_conv(x)
            return x

    def __init__(self, num_classes, in_channels=3, freeze_bn=False, sigmoid=True):
        super(UNetWithResnet18Encoder, self).__init__()
        self.sigmoid = sigmoid
        resnet18 = models.resnet18(pretrained=True)
        
        if in_channels != 3:
            resnet18.conv1 = nn.Conv2d(in_channels, 64, kernel_size=7, stride=2, padding=3, bias=False)

        self.encoder1 = nn.Sequential(resnet18.conv1, resnet18.bn1, resnet18.relu)
        self.encoder2 = resnet18.layer1
        self.encoder3 = resnet18.layer2
        self.encoder4 = resnet18.layer3
        self.encoder5 = resnet18.layer4

        self.up1 = UNetWithResnet18Encoder.Decoder(512, 256, 256)
        self.up2 = UNetWithResnet18Encoder.Decoder(256, 128, 128)
        self.up3 = UNetWithResnet18Encoder.Decoder(128, 64, 64)
        self.up4 = UNetWithResnet18Encoder.Decoder(64, 64, 64)

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
        x = F.interpolate(x, size=(x.size(2)*2, x.size(3)*2), mode='bilinear', align_corners=True)

        x = self.final_conv(x)
        
        if self.sigmoid:
            x = torch.sigmoid(x)
        return x

    def freeze_bn(self):
        for module in self.modules():
            if isinstance(module, nn.BatchNorm2d):
                module.eval()


class MultiLabelResNet(nn.Module):
    def __init__(self, num_labels, input_channels=3, sigmoid=True, pretrained=True,):
        super(MultiLabelResNet, self).__init__()
        self.model = models.resnet18(pretrained=pretrained)
        self.sigmoid = sigmoid

        if input_channels != 3:
            self.model.conv1 = nn.Conv2d(input_channels, 64, kernel_size=7, stride=2, padding=3, bias=False)
        
        num_ftrs = self.model.fc.in_features
        
        self.model.fc = nn.Linear(num_ftrs, num_labels)

    def forward(self, x):
        x = self.model(x)
        if self.sigmoid:
            x = torch.sigmoid(x)
        return x


class CombinedModel(nn.Module):
    def __init__(self, segment_model: nn.Module, predict_model: nn.Module, cat_layers:int=None):
        super(CombinedModel, self).__init__()
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

