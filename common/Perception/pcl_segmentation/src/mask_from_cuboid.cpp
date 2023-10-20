#include "mask_from_cuboid.h"
#include <pcl/filters/crop_box.h>
#include <pcl/common/eigen.h>
#include <pcl/point_types.h>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
#include <ros/console.h>
bool mask_from_cuboid(
	pcl::PointCloud<pcl::PointXYZ>::Ptr& in_cloud,
	sensor_msgs::PointCloud2& ros_cloud,
	geometry_msgs::Point& min,
	geometry_msgs::Point& max,
	sensor_msgs::Image& mask)
{
	pcl::CropBox<pcl::PointXYZ> cropBoxFilter(false);

	Eigen::Vector4f min_vec (min.x, min.y, min.z, 1.0);
	Eigen::Vector4f max_vec (max.x, max.y, max.z, 1.0);
	
	std::vector<int> indices;

	cropBoxFilter.setMin(min_vec);
	cropBoxFilter.setMax(max_vec);
	cropBoxFilter.setInputCloud(in_cloud);
	cropBoxFilter.filter(indices);

	mask.header.frame_id = ros_cloud.header.frame_id;
	mask.header.stamp = ros_cloud.header.stamp;
	mask.height = ros_cloud.height;
	mask.width = ros_cloud.width;
	mask.step = mask.width;
	mask.encoding = "mono8";
	mask.is_bigendian = 0;
	mask.data = std::vector<uint8_t>(mask.width * mask.height);
	for (const auto& idx : indices)
		mask.data[idx] = 255;
	return true;
}