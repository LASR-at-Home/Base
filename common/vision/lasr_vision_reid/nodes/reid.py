from typing import Dict, Tuple

import rospy
import torchreid
import torch
from torchvision import transforms
from PIL import Image as PILImage
import os
import rospkg
import numpy as np

Mat = np.ndarray


class ReID:

    dataset: str
    _model: torchreid.models.OSNet
    _device: torch.device
    _transform: transforms.Compose
    _db: Dict[str, torch.Tensor]

    def __init__(self, dataset: str):
        self._dataset = dataset
        self._dataset_root = os.path.join(
            rospkg.RosPack().get_path("lasr_vision_reid"), "datasets", self._dataset
        )
        self._model = torchreid.models.build_model("osnet_x1_0", pretrained=True)
        self._device = (
            torch.device("cuda") if torch.cuda.is_available() else torch.device("cpu")
        )
        self._model.to(self._device)
        self._model.eval()
        self._transform = transforms.Compose(
            [
                transforms.Resize((256, 128)),
                transforms.ToTensor(),
                transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
            ]
        )
        self._db = {}
        self._load_dataset()

    def _load_dataset(self):
        """
        Load all labeled face images from the dataset directory and extract features in batches.
        """
        if not os.path.exists(self._dataset_root):
            rospy.logerr(f"Dataset path not found: {self._dataset_root}")
            return

        rospy.loginfo(f"Loading dataset from: {self._dataset_root}")

        for label in os.listdir(self._dataset_root):
            label_path = os.path.join(self._dataset_root, label)
            if not os.path.isdir(label_path):
                continue

            image_tensors = []
            for fname in os.listdir(label_path):
                if not fname.lower().endswith(".png"):
                    continue

                img_path = os.path.join(label_path, fname)
                try:
                    pil_im = PILImage.open(img_path).convert("RGB")
                    x = self._transform(pil_im)
                    image_tensors.append(x)
                except Exception as e:
                    rospy.logwarn(f"Failed to process {img_path}: {e}")

            if not image_tensors:
                continue

            batch = torch.stack(image_tensors).to(self._device)
            with torch.no_grad():
                features = self._model(batch).cpu()

            self._db[label] = features

        rospy.loginfo(f"Loaded {len(self._db)} identities.")

    def _forward_model(self, cv_im: Mat) -> torch.Tensor:
        """
        Perform a forward pass of the model using the input image.
        """
        pil_im = PILImage.fromarray(cv_im)
        x = self._transform(pil_im).unsqueeze(0).to(self._device)
        with torch.no_grad():
            feat = self._model(x).cpu()
        return feat

    def _perform_reid(self, cv_im: Mat) -> Tuple[str, float]:
        """
        Lookup in the dataset to find the closest match.
        """


if __name__ == "__main__":
    rospy.init_node("lasr_vision_reid")
    dataset = rospy.get_param("lasr_vision_reid/dataset")
    reid = ReID(dataset)
    rospy.spin()
