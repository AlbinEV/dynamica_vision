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
#include <pcl/search/kdtree.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>



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
static rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr clustered_pcl;
static rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_pointcloud_sub;

bool static_background_saved = false;
int width_ref, height_ref;
float distance_threshold = 0.6;

pcl::PointCloud<PointT>::Ptr xyz_cloud(new pcl::PointCloud<PointT>);
PointCloudSM::Ptr bkg_cloud(new PointCloudSM);
PointCloudSM::Ptr frg_cloud(new PointCloudSM);
PointCloudSM::Ptr clustered_cloud(new PointCloudSM);
PCMsg static_output;
PCMsg dynamic_output;
PCMsg clusters_output;

double cluster_tolerance = 0.02; //2cm
int minimum_cluster_size = 500;

void clustering_step(PointCloudSM::Ptr filtered_cloud)
{
  	pcl::copyPointCloud(*filtered_cloud, *clustered_cloud);
  	clustered_cloud->clear();
  	    
	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
	tree->setInputCloud (filtered_cloud);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<PointT> ec;
	ec.setClusterTolerance (cluster_tolerance); // 2cm
	ec.setMinClusterSize (minimum_cluster_size);
	ec.setSearchMethod (tree);
	ec.setInputCloud (filtered_cloud);
	ec.extract (cluster_indices);
  	  
  	// Gives each cluster a random color
  	for(int i = 0; i < cluster_indices.size(); i++)
  	{
    		uint8_t r(rand()%255), g(rand()%255), b(rand()%255);
    		for(int j = 0; j < cluster_indices[i].indices.size(); j++)
    		{
      			filtered_cloud->points[cluster_indices[i].indices[j]].r = r;
      			filtered_cloud->points[cluster_indices[i].indices[j]].g = g;
      			filtered_cloud->points[cluster_indices[i].indices[j]].b = b;
      			clustered_cloud->push_back(filtered_cloud->points[cluster_indices[i].indices[j]]);
    		}

  	}
  
}


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
        //clustering_step(frg_cloud);
        pcl::toROSMsg(*frg_cloud, dynamic_output);
        pcl::toROSMsg(*clustered_cloud, clusters_output);
        dynamic_pcl->publish(dynamic_output);
        static_pcl->publish(static_output);
        //clustered_pcl->publish(clusters_output);
    }
}
 
int main(int argc, char ** argv)
{

  rclcpp::init(argc, argv);

  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("background_removal");
            
  auto camera_pointcloud_sub = node->create_subscription<sensor_msgs::msg::PointCloud2>("/robot/camera_1/point_cloud", 10, background_removal_callback);

  dynamic_pcl = node->create_publisher<sensor_msgs::msg::PointCloud2>("/robot/camera_1/dynamic_foreground", 10);
  static_pcl = node->create_publisher<sensor_msgs::msg::PointCloud2>("/robot/camera_1/static_background", 10);
  clustered_pcl = node->create_publisher<sensor_msgs::msg::PointCloud2>("/robot/camera_1/clusters", 10);
    
  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
