#include "segment_cuboid.h"
#include <pcl/filters/crop_box.h>
#include <pcl/common/eigen.h>
#include <pcl/point_types.h>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"

bool segment_cuboid(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr& in_cloud,
	geometry_msgs::Point& min,
	geometry_msgs::Point& max,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr& out_cloud
)
{
	pcl::CropBox<pcl::PointXYZRGB> cropBoxFilter(false);
	std::vector<int> indices;
	Eigen::Vector4f min_vec (min.x, min.y, min.z, 1.0);
	Eigen::Vector4f max_vec (max.x, max.y, max.z, 1.0);
	cropBoxFilter.setMin(min_vec);
	cropBoxFilter.setMax(max_vec);
	cropBoxFilter.setInputCloud(in_cloud);
	cropBoxFilter.filter(*out_cloud);

	return true;

}
