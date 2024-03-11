import torch
import torch.nn as nn
import torch.nn.functional as F
import torchvision.models as models


def X2conv(in_channels, out_channels, inner_channels=None):
    inner_channels = out_channels // 2 if inner_channels is None else inner_channels
    down_conv = nn.Sequential(
        nn.Conv2d(in_channels, inner_channels, kernel_size=3, padding=1, bias=False),
        nn.BatchNorm2d(inner_channels),
        nn.ReLU(inplace=True),
        nn.Conv2d(inner_channels, out_channels, kernel_size=3, padding=1, bias=False),
        nn.BatchNorm2d(out_channels),
        nn.ReLU(inplace=True))
    return down_conv


class Decoder(nn.Module):
    def __init__(self, in_channels, skip_channels, out_channels):
        super(Decoder, self).__init__()
        self.up = nn.ConvTranspose2d(in_channels, out_channels, kernel_size=2, stride=2)
        self.up_conv = X2conv(out_channels + skip_channels, out_channels)

    def forward(self, x_copy, x):
        x = self.up(x)
        if x.size(2) != x_copy.size(2) or x.size(3) != x_copy.size(3):
            x = F.interpolate(x, size=(x_copy.size(2), x_copy.size(3)), mode='bilinear', align_corners=True)
        x = torch.cat((x_copy, x), dim=1)
        x = self.up_conv(x)
        return x


class UNetWithResnetEncoder(nn.Module):
    def __init__(self, num_classes, in_channels=3, freeze_bn=False, sigmoid=True):
        super(UNetWithResnetEncoder, self).__init__()
        self.sigmoid = sigmoid
        self.resnet = models.resnet34(pretrained=False)  # Initialize with a ResNet model
        if in_channels != 3:
            self.resnet.conv1 = nn.Conv2d(in_channels, 64, kernel_size=7, stride=2, padding=3, bias=False)

        self.encoder1 = nn.Sequential(self.resnet.conv1, self.resnet.bn1, self.resnet.relu)
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
        x = F.interpolate(x, size=(x.size(2) * 2, x.size(3) * 2), mode='bilinear', align_corners=True)

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
    def __init__(self, num_labels, input_channels=3, sigmoid=True, pretrained=False,):
        super(MultiLabelResNet, self).__init__()
        self.model = models.resnet34(pretrained=False)
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
    def __init__(self, segment_model: nn.Module, predict_model: nn.Module, cat_layers: int=None):
        super(CombinedModel, self).__init__()
        self.segment_model = segment_model
        self.predict_model = predict_model
        self.cat_layers = cat_layers
        self.freeze_seg = False

    def forward(self, x: torch.Tensor):
        seg_masks = self.segment_model(x)
        seg_masks_ = seg_masks.detach()
        if self.cat_layers:
            seg_masks_ = seg_masks_[:, 0:self.cat_layers]
            x = torch.cat((x, seg_masks_), dim=1)
        else:
            x = torch.cat((x, seg_masks_), dim=1)
        logic_outputs = self.predict_model(x)
        return seg_masks, logic_outputs

    def freeze_segment_model(self):
        self.segment_model.eval()

    def unfreeze_segment_model(self):
        self.segment_model.train()
