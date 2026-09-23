cmake_minimum_required(VERSION 3.8)
project(bt-ros2-interface LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++ Standardı
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(xbot_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(demo
  # src/bt_sub_node.cpp
  src/bt_service_dinamic_node.cpp
  src/bt_sub_dinamic_node.cpp
  src/main.cpp
)

# Header yollarını hedefe bağlayın
target_include_directories(demo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# rclcpp ve behaviortree include/library yollarını demo hedefine aktarın
ament_target_dependencies(demo
  rclcpp
  std_msgs
  xbot_msgs
  sensor_msgs
  std_srvs
  behaviortree_cpp
)

install(TARGETS demo
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include

)
install(DIRECTORY config/
DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
