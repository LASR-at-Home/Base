#include "centroid.h"
#include <pcl/common/centroid.h>

bool centroid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointXYZRGB &centroid) {

    pcl::CentroidPoint<pcl::PointXYZRGB> centroid_point;
    for (auto it = cloud->begin(); it != cloud->end(); ++it)
        centroid_point.add(*it);
    centroid_point.get(centroid);
    return true;
}
