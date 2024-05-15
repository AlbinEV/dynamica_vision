#include <depthimage_to_pointcloud2/depth_conversions.hpp>
#include <depthimage_to_pointcloud2/depth_traits.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging_macros.h>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>



#include <limits>
#include <memory>
#include <string>
#include <vector>

static rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr g_pub_point_cloud;
static rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub;
static sensor_msgs::msg::CameraInfo::SharedPtr g_cam_info;
bool rgb_image_arrived = false, camera_info_arrived = false;
image_geometry::PinholeCameraModel camera_model;
static sensor_msgs::msg::Image::ConstSharedPtr rgb_msg;

template<typename T>
void convert_rgb(const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
             const sensor_msgs::msg::Image::ConstSharedPtr & rgb_msg,
             const sensor_msgs::msg::PointCloud2::SharedPtr & cloud_msg,
             int red_offset, 
             int green_offset, 
             int blue_offset, 
             int color_step)
{
  // Use correct principal point from calibration
  float center_x = camera_model.cx();
  float center_y = camera_model.cy();

  // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
  double unit_scaling = depthimage_to_pointcloud2::DepthTraits<T>::toMeters( T(1) );
  float constant_x = unit_scaling / camera_model.fx();
  float constant_y = unit_scaling / camera_model.fy();
  float bad_point = std::numeric_limits<float>::quiet_NaN ();

  const T* depth_row = reinterpret_cast<const T*>(&depth_msg->data[0]);
  int row_step = depth_msg->step / sizeof(T);
  const uint8_t* rgb = &rgb_msg->data[0];
  int rgb_skip = rgb_msg->step - rgb_msg->width * color_step;

  sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*cloud_msg, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*cloud_msg, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*cloud_msg, "b");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_a(*cloud_msg, "a");
  for (int v = 0; v < int(cloud_msg->height); ++v, depth_row += row_step, rgb += rgb_skip)
  {
    for (int u = 0; u < int(cloud_msg->width); ++u, rgb += color_step, ++iter_x, ++iter_y, ++iter_z, ++iter_a, ++iter_r, ++iter_g, ++iter_b)
    {
      T depth = depth_row[u];

      // Check for invalid measurements
      if (!depthimage_to_pointcloud2::DepthTraits<T>::valid(depth))
      {
        *iter_x = *iter_y = *iter_z = bad_point;
      }
      else
      {
        // Fill in XYZ
        *iter_x = (u - center_x) * depth * constant_x;
        *iter_y = (v - center_y) * depth * constant_y;
        *iter_z = depthimage_to_pointcloud2::DepthTraits<T>::toMeters(depth);
      }
      
      // Fill in color
      *iter_a = 255;
      *iter_r = rgb[red_offset];
      *iter_g = rgb[green_offset];
      *iter_b = rgb[blue_offset]; 
    }
  }
}


static void rgbCb(const sensor_msgs::msg::Image::ConstSharedPtr rgb_msg_in)
{
  rgb_msg = rgb_msg_in;
//   geometry_msgs::TransformStamped transformStamped;
//   transformStamped.header.stamp = ros::Time::now();
//   transformStamped.header.frame_id = "viper/left_camera_link";
//   transformStamped.child_frame_id = "/viper/cvm_base_link";
  rgb_image_arrived = true;
}

static void depthCb(const sensor_msgs::msg::Image::SharedPtr depth_image)
{

  if (camera_info_arrived == true && rgb_image_arrived == true)
  {
	  sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
	  cloud_msg->header = depth_image->header;
	  cloud_msg->height = depth_image->height;
	  cloud_msg->width = depth_image->width;
	  cloud_msg->is_dense = false;
	  cloud_msg->is_bigendian = false;
	  cloud_msg->fields.clear();
	  cloud_msg->fields.reserve(1);

	  sensor_msgs::PointCloud2Modifier pcd_modifier(*cloud_msg);
	  pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
	  //pcd_modifier.setPointCloud2FieldsByString(1, "xyz");

	  /*if (image->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
	    depthimage_to_pointcloud2::convert<uint16_t>(image, cloud_msg, camera_model);
	  } else if (image->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
	    depthimage_to_pointcloud2::convert<float>(image, cloud_msg, camera_model);
	  } else {
	    RCUTILS_LOG_WARN_THROTTLE(RCUTILS_STEADY_TIME, 5000,
	      "Depth image has unsupported encoding [%s]", image->encoding.c_str());
	    return;
	  }*/
	  
	  // Supported color encodings: RGB8, BGR8, MONO8
      	  int red_offset, green_offset, blue_offset, color_step;

          red_offset   = 0;
          green_offset = 1;
          blue_offset  = 2;
          color_step   = 3;
      
	  if (depth_image->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
	    convert_rgb<uint16_t>(depth_image, rgb_msg, cloud_msg, red_offset, green_offset, blue_offset, color_step);
	  } else if (depth_image->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
	    convert_rgb<float>(depth_image, rgb_msg, cloud_msg, red_offset, green_offset, blue_offset, color_step);
	  } else {
	    RCUTILS_LOG_WARN_THROTTLE(RCUTILS_STEADY_TIME, 5000,
	      "Depth image has unsupported encoding [%s]", depth_image->encoding.c_str());
	    return;
	  }

	  g_pub_point_cloud->publish(*cloud_msg);
   }
   
}

static void infoCb(sensor_msgs::msg::CameraInfo::SharedPtr info)
{
  g_cam_info = info;
  camera_model.fromCameraInfo(g_cam_info);
  cam_info_sub.reset();
  camera_info_arrived = true;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("depth2pcl");
            
  auto image_sub = node->create_subscription<sensor_msgs::msg::Image>("/robot/camera_1/depth/image_rect_raw", 10, depthCb);
  auto rgb_image_sub = node->create_subscription<sensor_msgs::msg::Image>("/robot/camera_1/color/image_raw", 10, rgbCb);
  cam_info_sub = node->create_subscription<sensor_msgs::msg::CameraInfo>("/robot/camera_1/depth/camera_info", 10, infoCb);

  g_pub_point_cloud = node->create_publisher<sensor_msgs::msg::PointCloud2>("/robot/camera_1/point_cloud", 10);
  
  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
