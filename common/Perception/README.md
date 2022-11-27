# The Perception Module

Core module for perception.

> It currently has multiple services:
1. face_detection -> openCV, (tbc and put in people detection folder, currently breaking)
2. lasr_object_detection_yolo
3. lasr_perception_server
4. people_detection
    - create_dataset
    - train_dataset_model 
    - recognise_people
    - (tbc) face_detection
5. robocup_recetionist - (tbc and cleaned up because it breaks), we have the TF transforms here

> The Core of the Perception module is the Perception service:

It takes:
```
sensor_msgs/Image[] image
string dataset
float32 confidence
float32 nms
string[] filter
string task
---
lasr_perception_server/Detection[] detected_objects
```

> The current tasks that it handles are:
1. yolo
2. open_cv
3. known_people - overlap of opencv and yolo (tbc in the future)

> Use it like that:

```python 
        det = rospy.ServiceProxy("lasr_perception_server/detect_objects_image", DetectImage)
        resp = det(imgs, "coco", 0.7, 0.3, ["person"], 'known_people').detected_objects
        resp = det(imgs, "coco", 0.7, 0.3, ["banana"], 'yolo').detected_objects
        resp = det(imgs, "coco", 0.7, 0.3, ["person"], 'open_cv').detected_objects
```