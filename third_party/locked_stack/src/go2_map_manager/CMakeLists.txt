cmake_minimum_required(VERSION 3.5)
project(go2_map_manager)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(small_gicp 1.0 REQUIRED CONFIG)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # ament_copyright only recognizes a fixed set of open-source licenses and
  # rejects this package's Proprietary license. Repository contract tests
  # enforce the GoGoGuard copyright header and package license instead.
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_executable(submap_builder
  src/submap_builder.cpp
)

add_executable(route_relocalizer
  src/route_relocalizer.cpp
)

add_executable(calibrated_cloud_transformer
  src/calibrated_cloud_transformer.cpp
)

add_executable(continuous_map_localizer
  src/continuous_map_localizer.cpp
)

add_executable(localization_registration_benchmark
  src/localization_registration_benchmark.cpp
)

ament_target_dependencies(submap_builder
  rclcpp
  sensor_msgs
  nav_msgs
  pcl_ros
  pcl_conversions
  tf2
  tf2_ros
)

target_link_libraries(submap_builder
  ${PCL_LIBRARIES}
)

ament_target_dependencies(route_relocalizer
  rclcpp
  sensor_msgs
  nav_msgs
  pcl_ros
  pcl_conversions
)

target_link_libraries(route_relocalizer
  ${PCL_LIBRARIES}
)

ament_target_dependencies(calibrated_cloud_transformer
  rclcpp
  sensor_msgs
  geometry_msgs
  pcl_conversions
  tf2
  tf2_ros
)

target_link_libraries(calibrated_cloud_transformer
  ${PCL_LIBRARIES}
)

ament_target_dependencies(continuous_map_localizer
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  std_srvs
  pcl_ros
  pcl_conversions
  tf2_ros
)

target_link_libraries(continuous_map_localizer
  ${PCL_LIBRARIES}
  small_gicp::small_gicp
)

target_link_libraries(localization_registration_benchmark
  ${PCL_LIBRARIES}
  small_gicp::small_gicp
)

install(TARGETS
  submap_builder
  route_relocalizer
  calibrated_cloud_transformer
  continuous_map_localizer
  localization_registration_benchmark
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
