#include "segment_bb.h"
#include <ros/console.h>

bool segment_bb(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr& in_cloud,
	int x1, int y1, int x2, int y2,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr& out_cloud)
{   
    for (auto y = y1; y < y2; ++y) 
        for (auto x = x1; x < x2; ++x)
            out_cloud->push_back(in_cloud->at(y,x));
}

