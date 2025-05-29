cmake_minimum_required(VERSION 3.5)
project(TF_Pub)

# 查找ROS 2包
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

# 添加可执行文件
add_executable(TF_node src/TF_node.cpp)

ament_target_dependencies(TF_node
  rclcpp 
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
)

# 链接库
target_link_libraries(TF_node
  Eigen3::Eigen
)

# 导出可执行文件
install(TARGETS
  TF_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()