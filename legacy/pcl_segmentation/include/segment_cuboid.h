#include <pcl_ros/point_cloud.h>
#include "geometry_msgs/Point.h"
#include <pcl/point_types.h>
#include "sensor_msgs/PointCloud2.h"

bool segment_cuboid(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr& in_cloud,
	geometry_msgs::Point& min,
	geometry_msgs::Point& max,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr& out_cloud
);