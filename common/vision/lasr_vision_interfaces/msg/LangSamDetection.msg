# Bounding box mask defined in pixel-space
int32[] xywh

# Segmentation mask defined in pixel-space
int32[] seg_mask

# Confidence score for detection
float32 detection_score

# Confidence score for segmentation mask
float32 seg_mask_score

# Point in target frame
geometry_msgs/Point point

# Name of detection
string name