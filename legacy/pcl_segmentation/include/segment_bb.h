#include <pcl_ros/point_cloud.h>
#include "geometry_msgs/Point.h"
#include <pcl/point_types.h>
#include "sensor_msgs/PointCloud2.h"

bool segment_bb(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr& in_cloud,
	int x1, int y1, int x2, int y2,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr& out_cloud
);
