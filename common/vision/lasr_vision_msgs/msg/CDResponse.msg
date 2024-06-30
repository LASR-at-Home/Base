# The combined masked image, if use_mask is True, else empty.
sensor_msgs/Image masked_img

# A list of all the cropped images of detections sorted according to the
# given method.
sensor_msgs/Image[] cropped_imgs

# A list of 2D detections, sorted to match the cropped_imgs
lasr_vision_msgs/Detection[] detections_2d

# A list of 3D detections, sorted to match the cropped_imgs
lasr_vision_msgs/Detection3D[] detections_3d

# Euclidian distance of given crop metric
float32[] distances

# IDs corresponding to which polygon(s) the detection centroid is contained in.
# An ID of 0 corresponds to the first polygon in the request, etc.
uint8[] polygon_ids

# The RGB image used for the 2D crop
sensor_msgs/Image rgb_image

# The pointcloud used for the 3D crop
sensor_msgs/PointCloud2 pointcloud