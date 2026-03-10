# Detected Object Class
string name

# How certain we are this is what we think it is
float32 confidence

# Bounding box mask defined in pixel-space
# X and Y are the midpoints of the bounding box.
int32[] xywh

# Segmentation mask defined in pixel-space
#
# This will be empty if a segmentation model was not used
int32[] xyseg
