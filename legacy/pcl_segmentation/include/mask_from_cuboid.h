#include <pcl_ros/point_cloud.h>
#include "geometry_msgs/Point.h"
#include <pcl/point_types.h>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"

bool mask_from_cuboid(
	pcl::PointCloud<pcl::PointXYZ>::Ptr& in_cloud,
	sensor_msgs::PointCloud2& ros_cloud,
	geometry_msgs::Point& min,
	geometry_msgs::Point& max,
	sensor_msgs::Image& mask);
