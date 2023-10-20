#ifndef PCL_SEGMENTATION_H
#define PCL_SEGMENTATION_H

#include "pcl_segmentation/SegmentCuboid.h"
#include "pcl_segmentation/SegmentBB.h"
#include "pcl_segmentation/Centroid.h"
#include "pcl_segmentation/MaskFromCuboid.h"

bool handle_segment_cuboid(pcl_segmentation::SegmentCuboid::Request& req, pcl_segmentation::SegmentCuboid::Response& res);
bool handle_segment_bb(pcl_segmentation::SegmentBB::Request& req, pcl_segmentation::SegmentBB::Response &res);
bool handle_mask_from_cuboid(pcl_segmentation::MaskFromCuboid::Request& req, pcl_segmentation::MaskFromCuboid::Response& res);
bool handle_centroid(pcl_segmentation::Centroid::Request& req, pcl_segmentation::Centroid::Response& res);

#endif // PCL_SEGMENTATION_H
