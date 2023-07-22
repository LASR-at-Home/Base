#include <ros/ros.h>
#include <ros/console.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/common/eigen.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"
#include "pcl_segmentation_server.h"
#include "segment_cuboid.h"
#include "mask_from_cuboid.h"
#include "centroid.h"
#include "segment_bb.h"

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
	ros::ServiceServer segment_bb_service = n.advertiseService("/pcl_segmentation_server/segment_bb", handle_segment_bb);
	ros::ServiceServer mask_from_cuboid_service = n.advertiseService("/pcl_segmentation_server/mask_from_cuboid", handle_mask_from_cuboid);
	ros::ServiceServer centroid_service = n.advertiseService("/pcl_segmentation_server/centroid", handle_centroid);
	ros::spin();
}

bool handle_segment_cuboid(pcl_segmentation::SegmentCuboid::Request& req, pcl_segmentation::SegmentCuboid::Response &res)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::fromROSMsg(req.points, *cloud);
	auto result =  segment_cuboid(cloud, req.min, req.max, cloud_out);
	pcl::toROSMsg(*cloud_out, res.points);
	res.points.header = req.points.header;
	return result;
}

bool handle_segment_bb(pcl_segmentation::SegmentBB::Request& req, pcl_segmentation::SegmentBB::Response &res)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::fromROSMsg(req.points, *cloud);
	segment_bb(cloud, req.x1, req.y1, req.x2, req.y2, cloud_out);
	pcl::toROSMsg(*cloud_out, res.points);
	res.points.header = req.points.header;
	return true;
}

bool handle_mask_from_cuboid(pcl_segmentation::MaskFromCuboid::Request& req, pcl_segmentation::MaskFromCuboid::Response &res)
{
	geometry_msgs::PointStamped min, max;
	sensor_msgs::PointCloud2::Ptr ros_cloud_tf(new sensor_msgs::PointCloud2);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tf(new pcl::PointCloud<pcl::PointXYZ>);
	min.point = req.min;
	min.header.frame_id = "map";
	min.header.stamp = req.points.header.stamp;
	max.point = req.max;
	max.header.frame_id = "map";
	max.header.stamp = req.points.header.stamp;
	
	sensor_msgs::PointCloud2 ros_cloud = req.points;

	geometry_msgs::TransformStamped transform = transformBuffer->lookupTransform(min.header.frame_id, req.points.header.frame_id, req.points.header.stamp, ros::Duration(2.0));
	tf2::doTransform(ros_cloud, *ros_cloud_tf, transform);	
	pcl::fromROSMsg(*ros_cloud_tf, *cloud_tf);

	auto result = mask_from_cuboid(cloud_tf, *ros_cloud_tf, min.point, max.point, res.mask);
	return result;
}

bool handle_centroid(pcl_segmentation::Centroid::Request& req, pcl_segmentation::Centroid::Response& res)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::fromROSMsg(req.points, *cloud);

	pcl::PointXYZRGB centroid_;

	auto result = centroid(cloud, centroid_);

	geometry_msgs::PointStamped point;
	point.header.frame_id = req.points.header.frame_id;
	point.header.stamp = req.points.header.stamp;
	point.point.x = centroid_.x;
	point.point.y = centroid_.y;
	point.point.z = centroid_.z;
	geometry_msgs::TransformStamped transform = transformBuffer->lookupTransform("map", req.points.header.frame_id, req.points.header.stamp, ros::Duration(2.0));
	tf2::doTransform(point, res.centroid, transform);
	return result;
}
