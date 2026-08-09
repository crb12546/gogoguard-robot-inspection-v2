cmake_minimum_required(VERSION 3.5)
project(go2_cmd_vel_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(unitree_sdk2 2.0.0 EXACT REQUIRED CONFIG)

add_executable(cmd_vel_udp_sender src/cmd_vel_udp_sender.cpp)
ament_target_dependencies(cmd_vel_udp_sender rclcpp geometry_msgs)
target_include_directories(cmd_vel_udp_sender PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(go2_sdk2_udp_receiver src/go2_sdk2_udp_receiver.cpp)
target_include_directories(go2_sdk2_udp_receiver PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(go2_sdk2_udp_receiver unitree_sdk2)

add_executable(go2_sdk2_motion_probe src/go2_sdk2_motion_probe.cpp)
target_link_libraries(go2_sdk2_motion_probe unitree_sdk2)

add_executable(go2_voice_control src/go2_voice_control.cpp)
target_link_libraries(go2_voice_control unitree_sdk2)

add_executable(go2_vui_control src/go2_vui_control.cpp)
target_link_libraries(go2_vui_control unitree_sdk2)

install(TARGETS
  cmd_vel_udp_sender
  go2_sdk2_udp_receiver
  go2_sdk2_motion_probe
  go2_voice_control
  go2_vui_control
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
