#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging_macros.h>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <pcl/PointIndices.h>
#include <pcl/common/impl/transforms.hpp>
#include "tf2/transform_datatypes.h"
#include "pcl_ros/transforms.hpp"
//#include <tf/transform_listener.h>
   
#include <limits>
#include <memory>
#include <string>
#include <vector>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudSM;
typedef sensor_msgs::msg::PointCloud2 PCMsg;

using namespace std;

static rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr merged_pointcloud;

bool pointcloud1_arrived = false;
int width_ref_1, height_ref_1;
int width_ref_2, height_ref_2;

PointCloudSM::Ptr cloud1(new PointCloudSM);
PointCloudSM::Ptr cloud2(new PointCloudSM);
PointCloudSM::Ptr merged_cloud(new PointCloudSM);

PCMsg output;

pcl::PCLPointCloud2 pcl_pc1;
pcl::PCLPointCloud2 pcl_pc2;
pcl::PCLPointCloud2 pcl_merged;
     
PCMsg p_msg;
 
PointCloudSM::Ptr cloud_transformed(new PointCloudSM);
PointCloudSM::Ptr temp_cloud(new PointCloudSM);

std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  
static void acquire_pointcloud1(const sensor_msgs::msg::PointCloud2::SharedPtr msg) 
{
	pcl::fromROSMsg(*msg, *cloud1);
        width_ref_1 = msg->width;
        height_ref_1 = msg->height;
        
        pcl_conversions::toPCL(*msg, pcl_pc1);
        
        pointcloud1_arrived = true;
}

static void acquire_pointcloud2(const sensor_msgs::msg::PointCloud2::SharedPtr msg) 
{
	if (pointcloud1_arrived == true)
	{
 	pcl::fromROSMsg(*msg, *cloud2);
        width_ref_2 = msg->width;
        height_ref_2 = msg->height;

	pcl::PCLPointCloud2 pcl_pc2;
	pcl_conversions::toPCL(*msg, pcl_pc2);

        pcl::PCLPointCloud2::concatenate(pcl_pc2, pcl_pc1, pcl_merged);
        pcl_conversions::fromPCL(pcl_merged, p_msg);
        
        merged_pointcloud->publish(p_msg);
        
        }
    	else
    	{
    	}
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("pcl_merging");
            
  auto camera1_pointcloud_sub = node->create_subscription<sensor_msgs::msg::PointCloud2>("/robot/camera_1/dynamic_foreground/transformed/point_cloud", 10, acquire_pointcloud1); //LEFT
  
  auto camera2_pointcloud_sub = node->create_subscription<sensor_msgs::msg::PointCloud2>("/robot/camera_2/dynamic_foreground", 10, acquire_pointcloud2); //TOP

  merged_pointcloud = node->create_publisher<sensor_msgs::msg::PointCloud2>("/robot/camera12/dynamic_foreground/point_cloud", 10);
  
  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
