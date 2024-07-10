# 2D methods are "centered", "left-most" "right-most" "top-most" "bottom-most"
# 3D methods are "closest" "furthest"
string method

# Whether to turn all pixels other than the YOLo mask to black
# If false, uses bounding-box cropping.
bool use_mask

# Name of the objects to get
string[] object_names

# Name of the yolo model to use for detections. If using mask is True,
# this must be a valid segementation model.
string yolo_model

# Confidence for YOLO model detections.
float32 yolo_model_confidence

# NMS threshold for YOLO model detections
float32 yolo_nms_threshold

# List of polygons to be used for 3D detections only
geometry_msgs/Polygon[] polygons        


# Whether to return the img or pointcloud that inference was run on
bool return_sensor_reading

# Optionally can give an RGB Image to run inference on
sensor_msgs/Image rgb_image

# Optionally can give a pointcloud to run inference on
sensor_msgs/PointCloud2 pointcloud