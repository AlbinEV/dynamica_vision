cmake_minimum_required(VERSION 3.5)
project(depth2pcl)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(image_geometry REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

add_executable(depth2pcl1
  src/depth2pcl1.cpp
)

add_executable(depth2pcl2
  src/depth2pcl2.cpp
)

ament_target_dependencies(depth2pcl1
  "image_geometry"
  "rclcpp"
  "sensor_msgs"
)

ament_target_dependencies(depth2pcl2
  "image_geometry"
  "rclcpp"
  "sensor_msgs"
)

install(TARGETS depth2pcl1
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS depth2pcl2
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
