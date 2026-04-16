cmake_minimum_required(VERSION 3.8)
project(auto_construct)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interface REQUIRED)
find_package(yaml-cpp REQUIRED)

find_package(nav2_msgs REQUIRED)       # Nav2 存图服务
find_package(nav_msgs REQUIRED)        # OccupancyGrid 地图消息
find_package(sensor_msgs REQUIRED)     # PointCloud2 点云消息
find_package(pcl_conversions REQUIRED) # ROS与PCL数据转换桥梁
find_package(PCL REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(rclcpp_action REQUIRED)
find_package(opennav_coverage_msgs REQUIRED)
find_package(OpenCV REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Pillar.msg"
  "msg/CoverageRegion.msg"
  "srv/ConfirmRegion.srv"
  "srv/SetRegion.srv"
  "srv/SetPathAndStart.srv"
  "srv/UpdateParams.srv"
  "srv/GetMapList.srv"
  DEPENDENCIES std_msgs geometry_msgs nav_msgs
)


include_directories(include)

add_executable(save_map_node 
  src/save_map.cpp
  
)

add_executable(mapping_manager_node 
  src/mapping_manager.cpp
)

add_executable(convert_map_node
  src/convert_map.cpp
)

add_executable(coverage_path_node
  src/coverage_path.cpp
)


ament_target_dependencies(save_map_node
  rclcpp
  std_srvs
  nav2_msgs
  interface
  nav_msgs
  sensor_msgs
  pcl_conversions
)

ament_target_dependencies(mapping_manager_node
  rclcpp
  std_srvs
  nav2_msgs
  interface
)

ament_target_dependencies(convert_map_node
  rclcpp
  rclcpp_action
  nav_msgs
  geometry_msgs
  opennav_coverage_msgs
  std_srvs
  nav2_msgs
  visualization_msgs
)

ament_target_dependencies(coverage_path_node
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
  std_msgs
  std_srvs
)

add_dependencies(convert_map_node ${PROJECT_NAME}__rosidl_typesupport_cpp)
add_dependencies(coverage_path_node ${PROJECT_NAME}__rosidl_typesupport_cpp)
add_dependencies(mapping_manager_node ${PROJECT_NAME}__rosidl_typesupport_cpp)

target_link_libraries(convert_map_node
  ${OpenCV_LIBS}
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

target_link_libraries(coverage_path_node
  yaml-cpp
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

target_link_libraries(mapping_manager_node
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)



target_link_libraries(save_map_node
  ${PCL_LIBRARIES}    
)

target_link_libraries(convert_map_node
  ${OpenCV_LIBS}
  yaml-cpp
)

install(TARGETS
  save_map_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  mapping_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  convert_map_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  coverage_path_node
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/ 
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


ament_export_dependencies(rosidl_default_runtime)

ament_package()
