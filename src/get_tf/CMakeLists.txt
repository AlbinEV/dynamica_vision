cmake_minimum_required(VERSION 3.8)
project(get_tf)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#add_compile_options(-std=c++11)
 
 
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(image_geometry REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL 1.12 REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED) 
find_package(pcl_ros REQUIRED)

include_directories(include SYSTEM ${PCL_COMMON_INCLUDE_DIRS} ${rs_driver_INCLUDE_DIRS})
include_directories(include)
include_directories(include ${PCL_INCLUDE_DIRS} )
include_directories(${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(get_tf
  src/get_tf.cpp 
)


ament_export_include_directories(include)

ament_target_dependencies(get_tf
  "image_geometry"
  "pcl_conversions"
  "rclcpp"
  "sensor_msgs"
  "pcl_ros"
  "tf2"
  "tf2_ros"
  "geometry_msgs"
    
    )

install(TARGETS get_tf
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
