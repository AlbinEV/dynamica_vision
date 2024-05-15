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
#include <limits>
#include <memory>
#include <string>
#include <vector>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudSM;
typedef sensor_msgs::msg::PointCloud2 PCMsg;

using namespace std;

static rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr dynamic_pcl;
static rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr static_pcl;
static rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_pointcloud_sub;

bool static_background_saved = false;
int width_ref, height_ref;
float distance_threshold = 0.6;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyz_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
PointCloudSM::Ptr bkg_cloud(new PointCloudSM);
PointCloudSM::Ptr frg_cloud(new PointCloudSM);
PCMsg static_output;
PCMsg dynamic_output;

void subtract_static_background(PointCloudSM::Ptr cloud)
{
    //std::cout << "Foreground generation . . ." << std::endl;
    frg_cloud->clear();
    frg_cloud->header = cloud->header;
        
    float distance = 0.0;

    for (int x = 0; x < width_ref; ++x)
    {
      for (int y = 0; y < height_ref; ++y)
      {
            //std::cout << "Distance computation . . ." << std::endl;
            PointT tmpCurr = cloud->at(x,y);
            //std::cout << "tmpCurr: " << tmpCurr << std::endl;
            PointT tmpBK = bkg_cloud->at(x,y);
            distance = sqrt(pow(tmpCurr.x-tmpBK.x, 2) + pow(tmpCurr.y-tmpBK.y, 2) + pow(tmpCurr.z-tmpBK.z, 2));
            //std::cout << "distance: " << distance << std::endl;
            if (distance > distance_threshold)
            	frg_cloud->push_back(tmpCurr);
      }
    }
  			 
}

static void background_removal_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) 
{
    if (static_background_saved == false)
    {
    	std::cout << "Background acquisition . . ." << std::endl;
        pcl::fromROSMsg(*msg, *bkg_cloud);
        width_ref = msg->width;  // 640
        height_ref = msg->height;  // 360
        pcl::toROSMsg(*bkg_cloud, static_output);
        std::cout << "Background is: " << width_ref << "x" << height_ref << std::endl;
        static_background_saved = true;
        }
    else
    {
    	//std::cout << "Background subtraction . . ." << std::endl;
        pcl::fromROSMsg(*msg, *xyz_cloud);
        subtract_static_background(xyz_cloud);
        pcl::toROSMsg(*frg_cloud, dynamic_output);
        dynamic_pcl->publish(dynamic_output);
        static_pcl->publish(static_output);
    }
}
 
int main(int argc, char ** argv)
{

  rclcpp::init(argc, argv);

  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("background_removal");
            
  auto camera_pointcloud_sub = node->create_subscription<sensor_msgs::msg::PointCloud2>("/robot/camera_2/point_cloud", 10, background_removal_callback);

  dynamic_pcl = node->create_publisher<sensor_msgs::msg::PointCloud2>("/robot/camera_2/dynamic_foreground", 10);
  static_pcl = node->create_publisher<sensor_msgs::msg::PointCloud2>("/robot/camera_2/static_background", 10);
    
  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
