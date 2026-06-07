from copy import deepcopy
from typing import Any, List, Tuple, Union

import cv2
import numpy as np
import onnxruntime as ort
import torch
import torch.nn.functional as F
import torchvision.transforms as transforms
from torchvision.transforms.functional import resize


class SamResize:
    def __init__(self, size: int) -> None:
        self.size = size

    def __call__(self, image: torch.Tensor) -> torch.Tensor:
        h, w, _ = image.shape
        long_side = max(h, w)
        if long_side != self.size:
            return self.apply_image(image)
        else:
            return image.permute(2, 0, 1)

    def apply_image(self, image: torch.Tensor) -> torch.Tensor:
        target_size = self.get_preprocess_shape(
            image.shape[0], image.shape[1], self.size
        )
        return resize(image.permute(2, 0, 1), target_size)

    @staticmethod
    def get_preprocess_shape(
        oldh: int, oldw: int, long_side_length: int
    ) -> Tuple[int, int]:
        scale = long_side_length * 1.0 / max(oldh, oldw)
        newh, neww = oldh * scale, oldw * scale
        return (int(newh + 0.5), int(neww + 0.5))


class SamEncoder:
    def __init__(self, model_path: str, device: str = "cpu", **kwargs):
        opt = ort.SessionOptions()
        provider = (
            ["CUDAExecutionProvider"] if device == "cuda" else ["CPUExecutionProvider"]
        )
        self.session = ort.InferenceSession(
            model_path, opt, providers=provider, **kwargs
        )
        self.input_name = self.session.get_inputs()[0].name

    def __call__(self, img: np.ndarray, *args: Any, **kwargs: Any) -> np.ndarray:
        return self.session.run(None, {self.input_name: img})[0]


class SamDecoder:
    def __init__(
        self,
        model_path: str,
        device: str = "cpu",
        target_size: int = 1024,
        mask_threshold: float = 0.0,
        **kwargs,
    ):
        opt = ort.SessionOptions()
        provider = (
            ["CUDAExecutionProvider"] if device == "cuda" else ["CPUExecutionProvider"]
        )
        self.target_size = target_size
        self.mask_threshold = mask_threshold
        self.session = ort.InferenceSession(
            model_path, opt, providers=provider, **kwargs
        )

    @staticmethod
    def get_preprocess_shape(
        oldh: int, oldw: int, long_side_length: int
    ) -> Tuple[int, int]:
        scale = long_side_length * 1.0 / max(oldh, oldw)
        return (int(oldh * scale + 0.5), int(oldw * scale + 0.5))

    def apply_coords(self, coords, original_size, new_size):
        old_h, old_w = original_size
        new_h, new_w = new_size
        coords = deepcopy(coords).astype(float)
        coords[..., 0] *= new_w / old_w
        coords[..., 1] *= new_h / old_h
        return coords

    def apply_boxes(self, boxes, original_size, new_size):
        return self.apply_coords(boxes.reshape(-1, 2, 2), original_size, new_size)

    def run(
        self,
        img_embeddings,
        origin_image_size,
        point_coords=None,
        point_labels=None,
        boxes=None,
        return_logits=False,
    ):
        input_size = self.get_preprocess_shape(
            *origin_image_size, long_side_length=self.target_size
        )

        if point_coords is None and point_labels is None and boxes is None:
            raise ValueError("Provide at least one box or point.")
        if img_embeddings.shape != (1, 256, 64, 64):
            raise ValueError("Wrong embedding shape.")

        if point_coords is not None:
            point_coords = self.apply_coords(
                point_coords, origin_image_size, input_size
            ).astype(np.float32)

        if boxes is not None:
            boxes = self.apply_boxes(boxes, origin_image_size, input_size).astype(
                np.float32
            )
            point_coords = boxes
            point_labels = np.array(
                [[2, 3] for _ in range(boxes.shape[0])], dtype=np.float32
            ).reshape((-1, 2))

        low_res_masks, iou_predictions = self.session.run(
            None,
            {
                "image_embeddings": img_embeddings,
                "point_coords": point_coords,
                "point_labels": point_labels,
            },
        )

        import sys

        print(
            f"[SAM] origin_image_size={origin_image_size}, input_size={input_size}, low_res_masks shape={low_res_masks.shape}, low_res min={low_res_masks.min():.3f} max={low_res_masks.max():.3f} mean={low_res_masks.mean():.3f}, point_coords={point_coords.tolist() if point_coords is not None else None}",
            flush=True,
        )
        sys.stdout.flush()
        masks = _mask_postprocessing(
            low_res_masks, origin_image_size, img_size=self.target_size
        )
        if not return_logits:
            masks = masks > self.mask_threshold
        return masks, iou_predictions, low_res_masks


def _mask_postprocessing(
    masks: np.ndarray, orig_im_size, img_size: int = 1024
) -> np.ndarray:
    masks = torch.tensor(masks, dtype=torch.float32)
    orig_im_size = torch.tensor(orig_im_size, dtype=torch.int64)
    masks = F.interpolate(
        masks, size=(img_size, img_size), mode="bilinear", align_corners=False
    )
    prepadded_size = _resize_longest_image_size(orig_im_size, img_size)
    masks = masks[..., : int(prepadded_size[0]), : int(prepadded_size[1])]
    h, w = orig_im_size[0], orig_im_size[1]
    masks = F.interpolate(masks, size=(h, w), mode="bilinear", align_corners=False)
    return masks.numpy()


def _resize_longest_image_size(
    input_image_size: torch.Tensor, longest_side: int
) -> torch.Tensor:
    scale = longest_side / torch.max(input_image_size.to(torch.float32))
    return torch.floor(scale * input_image_size.to(torch.float32) + 0.5).to(torch.int64)


class VitSam:
    def __init__(self, encoder_path: str, decoder_path: str, device: str = "cuda"):
        self.device = device
        self.encoder = SamEncoder(encoder_path, device=device)
        self.decoder = SamDecoder(decoder_path, device=device, target_size=1024)

    def __call__(
        self, image: np.ndarray, bboxes: List
    ) -> Tuple[np.ndarray, np.ndarray]:
        raw_img = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        origin_image_size = raw_img.shape[:2]
        img = self._preprocess(raw_img)
        img_embeddings = self.encoder(img)
        boxes = np.array(bboxes, dtype=np.float32)
        masks, _, _ = self.decoder.run(
            img_embeddings=img_embeddings,
            origin_image_size=origin_image_size,
            boxes=boxes,
        )
        return masks, boxes

    def _preprocess(self, x: np.ndarray, img_size: int = 512) -> np.ndarray:
        pixel_mean = [123.675 / 255, 116.28 / 255, 103.53 / 255]
        pixel_std = [58.395 / 255, 57.12 / 255, 57.375 / 255]
        x = torch.tensor(x)
        x = SamResize(img_size)(x).float() / 255
        x = transforms.Normalize(mean=pixel_mean, std=pixel_std)(x)
        h, w = x.shape[-2:]
        x = F.pad(x, (0, img_size - w, 0, img_size - h), value=0).unsqueeze(0).numpy()
        return x
