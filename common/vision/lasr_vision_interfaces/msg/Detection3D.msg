# Detected Object Class
string name

# How certain we are this is what we think it is
float32 confidence

# Bounding box mask defined in pixel-space
int32[] xywh

# Target frame
string target_frame

# Segmentation mask defined in pixel-space
#
# This will be empty if a segmentation model was not used
int32[] xyseg

# Point in map frame
geometry_msgs/Point point