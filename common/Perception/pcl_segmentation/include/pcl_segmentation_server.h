#ifndef PCL_SEGMENTATION_H
#define PCL_SEGMENTATION_H

#include "pcl_segmentation/SegmentCuboid.h"
#include "pcl_segmentation/Centroid.h"

bool handle_segment_cuboid(pcl_segmentation::SegmentCuboid::Request& req, pcl_segmentation::SegmentCuboid::Response& res);

bool handle_centroid(pcl_segmentation::Centroid::Request& req, pcl_segmentation::Centroid::Response& res);

#endif // PCL_SEGMENTATION_H
