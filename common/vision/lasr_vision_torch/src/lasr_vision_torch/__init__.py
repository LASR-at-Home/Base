from torch_module.modules import DeepLabV3PlusMobileNetV3, MultiLabelMobileNetV3Large, CombinedModelNoRegression
from torch_module.helpers import load_torch_model

import rospkg
from os import path


def load_face_classifier_model():
    cat_layers = 4
    # 'cloth', 'hair', 'hat', 'glasses', 'face',
    segment_model = DeepLabV3PlusMobileNetV3(num_classes=4)
    # 'hair', 'hat', 'glasses', 'face', ; first three with colours, rgb
    predict_model = MultiLabelMobileNetV3Large(cat_layers, 7)
    model = CombinedModelNoRegression(
        segment_model, predict_model, cat_layers=cat_layers)
    model.eval()

    r = rospkg.RosPack()
    model, _, _, _ = load_torch_model(model, None, path=path.join(r.get_path(
        "lasr_vision_torch"), "models", "best_model_epoch_31.pth"), cpu_only=True)
    return model
