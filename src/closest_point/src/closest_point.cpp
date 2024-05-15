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

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include <chrono>
#include <functional>
#include <memory>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include <limits>
#include <memory>
#include <string>
#include <vector>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudSM;
typedef sensor_msgs::msg::PointCloud2 PCMsg;

using std::placeholders::_1;
using namespace std;

pcl::PointCloud<PointT>::Ptr closest_cloud(new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr current_cloud(new pcl::PointCloud<PointT>);
PointCloudSM::Ptr clustered_cloud(new PointCloudSM);
PCMsg clusters_output;

std::vector<pcl::PointIndices> cluster_indices;
Eigen::Matrix4f T_tool0_camera;

geometry_msgs::msg::PointStamped pc_pt_min; 
geometry_msgs::msg::PointStamped robot_pt;
                                      
double cluster_tolerance = 0.02; //2cm
double min_dist = 0.0;
int minimum_cluster_size = 500;

void euclidean_clusters_extraction(PointCloudSM::Ptr current_cloud)
  {
  	pcl::copyPointCloud(*current_cloud, *clustered_cloud);
  	clustered_cloud->points.clear();

	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
	tree->setInputCloud (current_cloud);
	
	pcl::EuclideanClusterExtraction<PointT> ec;
	ec.setClusterTolerance (cluster_tolerance); // 2cm
	ec.setMinClusterSize (minimum_cluster_size);
	ec.setSearchMethod (tree);
	ec.setInputCloud (current_cloud);
	ec.extract (cluster_indices);

  	// Gives each cluster a random color
  	for(int i = 0; i < cluster_indices.size(); i++)
  	{
    		uint8_t r(rand()%255), g(rand()%255), b(rand()%255);
    		for(int j = 0; j < cluster_indices[i].indices.size(); j++)
    		{
      			current_cloud->points[cluster_indices[i].indices[j]].r = r;
      			current_cloud->points[cluster_indices[i].indices[j]].g = g;
      			current_cloud->points[cluster_indices[i].indices[j]].b = b;
      			clustered_cloud->push_back(current_cloud->points[cluster_indices[i].indices[j]]);
    		}
  	}
  
  }
  
  
class ClosestPointGenerator : public rclcpp::Node
{
public:
  ClosestPointGenerator()
  : Node("closest_point")
  {
    pointcloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/robot/camera_1/dynamic_foreground", 10, std::bind(&ClosestPointGenerator::clustering_callback, this, _1));
    clustered_pcl = this->create_publisher<sensor_msgs::msg::PointCloud2>("/robot/camera/clusters", 1);
      
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

private:

  void clustering_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
  {
  	//cout << "ciao" << endl;
	pcl::fromROSMsg(*msg, *current_cloud);
        euclidean_clusters_extraction(current_cloud);
        
        compute_tf_transf("camera1", "tool0", T_tool0_camera);
        get_closest_cluster_to_one_frame(	current_cloud, 
                                         	cluster_indices, 
                                      		T_tool0_camera,
                                      		"tool0", 
                                      		closest_cloud, 
                                      		min_dist, 
                                      		pc_pt_min, 
                                      		robot_pt);
                                      		
        pcl::toROSMsg(*clustered_cloud, clusters_output);
        clustered_pcl->publish(clusters_output);
  }

  void compute_tf_transf(std::string reference_frame, std::string frame2transform, Eigen::Matrix4f& roto_transl_matrix) const
  {
        geometry_msgs::msg::TransformStamped t;

        try {
          t = tf_buffer_->lookupTransform(reference_frame, frame2transform, tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
          RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            reference_frame, frame2transform, ex.what());
          return;
        }

  	/*cout << "calcolo..." << endl;
  	cout << t.transform.translation.x << endl;
  	cout << t.transform.translation.y << endl;
  	cout << t.transform.translation.z << endl;
  	
  	cout << t.transform.rotation.x << endl;
  	cout << t.transform.rotation.y << endl;
  	cout << t.transform.rotation.z << endl;
  	cout << t.transform.rotation.w << endl;*/
  	
  	tf2::Quaternion q(t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w);
  	tf2::Matrix3x3 m(q);
  	roto_transl_matrix << m[0][0], m[0][1], m[0][2], t.transform.translation.x, 
                        m[1][0], m[1][1], m[1][2], t.transform.translation.y, 
                        m[2][0], m[2][1], m[2][2], t.transform.translation.z, 
                            0.0,     0.0,     0.0,                          1.0;
        
        //cout << "roto_transl_matrix..." << endl;
        //cout << roto_transl_matrix << endl;
                                   
  }
  

void get_closest_cluster_to_one_frame(PointCloudSM::Ptr pc_in,
				      //boost::shared_ptr<pcl::PointCloud<PointT> > pc_in, 
                                      std::vector<pcl::PointIndices> cluster_indices, 
                                      Eigen::Matrix4f T_matrix,
                                      std::string frame_id, 
                                      PointCloudSM::Ptr pc_out,
                                      //boost::shared_ptr<pcl::PointCloud<PointT> >& pc_out, 
                                      double& min_dist, 
                                      geometry_msgs::msg::PointStamped& pc_pt_min, 
                                      geometry_msgs::msg::PointStamped& robot_pt) const
{

  // Check there is at least 1 cluster
  if ((cluster_indices.size() < 1))
    return;

  std::vector<double> min_dists;   
  std::vector<int> cluster_id;
  typename pcl::PointCloud<PointT>::Ptr temp_pc(new pcl::PointCloud<PointT>);
  typename std::vector< pcl::PointCloud<PointT> > temp_pcs;
  std::vector< geometry_msgs::msg::PointStamped> pc_pt_mins;

  for(int i = 0; i < cluster_indices.size(); i++){
    cluster_id.clear();
    cluster_id.push_back(i);
     
    //extract_clusters_from_pc(pc_in, cluster_indices, cluster_id, temp_pc);
    //compute_pc2frame_min_dist(temp_pc, T_matrix, frame_id, min_dist, pc_pt_min, robot_pt);
    compute_pc2frame_min_dist(pc_in, T_matrix, frame_id, min_dist, pc_pt_min, robot_pt);
     
    temp_pcs.push_back(*temp_pc);       // analyzed cluster [pcd]
    min_dists.push_back(min_dist);      // minimum distance of the analyzed cluster in respect to 'frame_id' [m]
    pc_pt_mins.push_back(pc_pt_min);    // point of the analyzed cluster at minimum distance in respect to 'frame_id' [PointStamped]
  }
  
  // Selection of the (closest) minimum distance point inside the 'min_dists' vector
  int min_id = 0;
  min_dist = min_dists[0];
  for(int i = 0; i < min_dists.size();i++){
    if(min_dists[i] < min_dist){
      min_dist = min_dists[i];          
      min_id = i;
    } 
  }
  
  *pc_out = temp_pcs[min_id];
  pc_pt_min = pc_pt_mins[min_id];
  cout << "Min dist: " << min_dist << endl;
}

void compute_pc2frame_min_dist( PointCloudSM::Ptr current_cloud,
				//boost::shared_ptr<pcl::PointCloud<PointT> > pc, 
                                Eigen::Matrix4f T_matrix, 
                                std::string frame_id,
                                double& min_dist, 
                                geometry_msgs::msg::PointStamped& pc_pt_min, 
                                geometry_msgs::msg::PointStamped& robot_pt) const
{

  Eigen::Vector3f pt2( T_matrix(0, 3), T_matrix(1, 3), T_matrix(2, 3));

  // Selection of the minimum distance point of the analyzed cluster 'pc'
  min_dist = 100000000; // [m]
  float dist = 0; 
  int min_idx = 0;
  for (size_t i = 0; i < pc->points.size (); ++i){
    pcl::Vector3fMap pt1 = pc->points[i].getVector3fMap();
    dist = (pt2 - pt1).norm ();
    if (dist < min_dist){
      min_idx = i;
      min_dist = dist;
    }
  }

  pc_pt_min.header.frame_id = pc->header.frame_id;
  pc_pt_min.point.x = pc->points[min_idx].x;
  pc_pt_min.point.y = pc->points[min_idx].y;
  pc_pt_min.point.z = pc->points[min_idx].z; 

  //Ricavo il punto sulla sfera:
  Eigen::Vector4f pi_k(pc_pt_min.point.x, pc_pt_min.point.y, pc_pt_min.point.z, 1.0);
  Eigen::Matrix4f Tkl = T_matrix.inverse();
  Eigen::Vector4f pi_l;
  pi_l = Tkl*pi_k;

  Eigen::Vector3f pi_l3;
  pi_l3[0] = pi_l[0];
  pi_l3[1] = pi_l[1];
  pi_l3[2] = pi_l[2];

  float norma_pil = 0; 
  norma_pil = (pi_l3).norm ();

  Eigen::Vector3f qi_l;
  qi_l = pi_l3*0.05/norma_pil;

  robot_pt.header.frame_id = frame_id;
  robot_pt.point.x = qi_l[0];
  robot_pt.point.y = qi_l[1];
  robot_pt.point.z = qi_l[2];

}



/*
void extract_clusters_from_pc(boost::shared_ptr<pcl::PointCloud<PointT> > pc_in, 
                              std::vector<pcl::PointIndices> cluster_indices, 
                              std::vector<int> cluster_ids, 
                              boost::shared_ptr<pcl::PointCloud<PointT2> >& pc_out) const
{
  
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
  for(int i=0; i<cluster_ids.size(); i++){
    for(int j=0; j<cluster_indices[cluster_ids[i]].indices.size(); j++)
      inliers->indices.push_back(cluster_indices[cluster_ids[i]].indices[j]);
  }
  
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud (pc_in);
  extract.setIndices (inliers);
  extract.setNegative (false);
  extract.filter (*pc_out);
  
  pc_out->header.frame_id = pc_in->header.frame_id;
}*/


  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr clustered_pcl{nullptr};
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub;
      
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};






int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ClosestPointGenerator>());
  rclcpp::shutdown();

  return 0;
}
