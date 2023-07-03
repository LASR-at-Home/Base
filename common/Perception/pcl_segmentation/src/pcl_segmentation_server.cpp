#include <ros/ros.h>
#include <ros/console.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/common/eigen.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"
#include "pcl_segmentation_server.h"
#include "segment_cuboid.h"

tf2_ros::Buffer * transformBuffer;
tf2_ros::TransformListener * transformListener;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "pcl_segmentation_server");
	transformBuffer = new tf2_ros::Buffer(ros::Duration(30.0));
	transformListener = new tf2_ros::TransformListener(*transformBuffer);
	ros::Duration(2).sleep();
	
	ROS_INFO("Segmentation server started");

	ros::NodeHandle n;
	ros::ServiceServer segment_cuboid_service = n.advertiseService("/pcl_segmentation_server/segment_cuboid", handle_segment_cuboid);

	ros::spin();
}

bool handle_segment_cuboid(pcl_segmentation::SegmentCuboid::Request& req, pcl_segmentation::SegmentCuboid::Response &res)
{
	ROS_INFO("segment_cuboid called");
	geometry_msgs::PointStamped min, max;
	sensor_msgs::PointCloud2::Ptr ros_cloud_tf(new sensor_msgs::PointCloud2);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tf(new pcl::PointCloud<pcl::PointXYZ>);

	min.point = req.min;
	min.header.frame_id = "map";
	min.header.stamp = req.points.header.stamp;
	max.point = req.max;
	max.header.frame_id = "map";
	max.header.stamp = req.points.header.stamp;
	
	sensor_msgs::PointCloud2& ros_cloud = req.points;

	geometry_msgs::TransformStamped transform = transformBuffer->lookupTransform(min.header.frame_id, req.points.header.frame_id, req.points.header.stamp, ros::Duration(2.0));
	tf2::doTransform(ros_cloud, *ros_cloud_tf, transform);	
	pcl::moveFromROSMsg(*ros_cloud_tf, *cloud_tf);

/*	transformListener->transformPoint(req.points.header.frame_id, min, minT);
	transformListener->transformPoint(req.points.header.frame_id, max, maxT); 
	ROS_INFO_STREAM("Transformed points to camera frame" << minT.point.x << " " << minT.point.y << " " << minT.point.z << " " << maxT.point.x << " " << maxT.point.y << " " << maxT.point.z); */

	ROS_INFO_STREAM("PCL SIZE " << cloud_tf->size());
	auto result = segment_cuboid(cloud_tf, *ros_cloud_tf, min.point, max.point, res.mask);
	ROS_INFO_STREAM("IMG SIZE " << res.mask.width * res.mask.height);
	return result;
}
