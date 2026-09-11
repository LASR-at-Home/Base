# Vision
This folder contains vision packages which provides yolo based services.

## lasr_vision_interfaces
This package containes `msg`, `srv`, and `action` definitions for custom vision components.
### What it provides:
  - Messages: `Detection`, `Detection3D`, `Detection3DArray`, `Keypoint`, `Keypoint3D`, `KeypointList`, `Keypoint3DList`
  - Services:  
    - `YoloDetection` — service type for 2D YOLO detection  
    - `YoloDetection3D` — service type for 3D YOLO detection  
    - `YoloPoseDetection`, `YoloPoseDetection3D` — pose/keypoint services  
    - `OpenVocabDetect`, `OpenVocabDetectAndSegment` — open-vocab detection services  
    - `Recognise3D`, `AddFace` — re-id services  
    - `VlmDescribePeople` — VLM describe-person service  

  - Actions
    - `EyeTracker` — eye-tracker action

## lasr_vision_yolo
This is a core package providing YOLO-based item and pose detection services.
### What it provides:
  - Services:
    - `YoloDetection` — `/yolo/detect`
    - `YoloDetection3D` — `/yolo/detect3d`
    - `YoloPoseDetection` — `/yolo/detect_pose`
    - `YoloPoseDetection3D` — `/yolo/detect3d_pose`
  - Published topics:
    - Annotated images per-model: `/yolo/detect/<model>` (image publisher)
    - 3D detection markers per-model: `/yolo/detect3d/<model>` (Marker)
    - Pose 3D markers: `/yolo/pose3d/<model>` (MarkerArray)
### Notes
 - For use in simulation disable the dept conversion by commenting out the `/ 1000.0` in the service node.

## lasr_vision_open_vocabulary
This is a Open-vocabulary detection node wrapping models like Grounding DINO and YOLO-World; optional SAM segmentation support.
### What it provides:
  - Services:
    - `OpenVocabDetect` — `open_vocab/detect`
    - `OpenVocabDetectAndSegment` — `open_vocab/detect_and_segment`
  - Visualization / outputs:
    - Marker array: `/detection_markers`
    - Centroid detections: `/object_centroids` (Detection3DArray)
  - Example client: visualizer node calls `open_vocab/detect` to produce markers & centroids.
### Notes
  - Optionally uses ViT-SAM for segmentation; segmentation outputs returned as images for `detect_and_segment`.



## lasr_vision_reid
This package provides face re-identification service using DeepFace embeddings and an in-memory gallery.
### What it provides:
  - Services:
    - `Recognise3D` — `/lasr_vision_reid/recognise` (returns 3D detections when depth available)
    - `AddFace` — `/lasr_vision_reid/add_face` (add embedding to in-memory DB)
  - Published topics:
    - Annotated image: `/lasr_vision_reid/recognise/detections` (Image)
    - Recognised point markers: `/lasr_vision_reid/recognise/points` (Marker)
### Notes
  - Embeddings are kept in-memory (no persistence by default).



## lasr_vision_eye_tracker
This is a action server that tracks eyes and commands head controllers to look at detected eye centroids.
### What it provides:
  - Action:
    - `EyeTracker` — `/lasr_vision_eye_tracker/track_eyes`
### Notes:
  - Requires head controller internal package 


## lasr_vlm
This is a Visual Language Model package to describe people and return attributes via a service.
### What it provides:
  - Service:
    - `VlmDescribePeople` — `/vlm/describe_people`
  - Outputs: attributes such as `hair_color`, `hair_length`, `glasses`, `hat`, `shirt_color`.
### Notes
  - This can be extended to perform other VLM inferences.