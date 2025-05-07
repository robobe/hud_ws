cmake_minimum_required(VERSION 3.8)
project(rome_hud)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_subdirectory(examples)



# find dependencies
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rome_interfaces REQUIRED)

pluginlib_export_plugin_description_file(rome_hud hud_plugins.xml)

include_directories(include ${OpenCV_INCLUDE_DIRS} ${rclcpp_INCLUDE_DIRS} ${rome_interfaces_INCLUDE_DIRS})


add_library(rome_hud SHARED
  src/plugins/tracker_hud_item.cpp
  src/plugins/master_caution.cpp
  src/plugins/fid.cpp
  src/plugins/sm_status.cpp
)
target_compile_features(rome_hud PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  rome_hud
  "pluginlib"
  OpenCV
  rclcpp
  std_msgs 
  rome_interfaces
)

install(TARGETS rome_hud
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# \=========================

set(plugins_dependencies
  pluginlib
)

add_executable(hud_node src/hud.cpp)
target_include_directories(hud_node PRIVATE
  ${OpenCV_INCLUDE_DIRS} 
  ${rclcpp_INCLUDE_DIRS}
)
ament_target_dependencies(hud_node 
  rclcpp 
  ${plugins_dependencies} 
  OpenCV
  sensor_msgs 
  cv_bridge)



add_executable(hud_tester src/hud_tester.cpp)
target_include_directories(hud_tester PRIVATE
  ${OpenCV_INCLUDE_DIRS} 
  ${rclcpp_INCLUDE_DIRS}
)
ament_target_dependencies(hud_tester rclcpp ${plugins_dependencies} OpenCV)

install(TARGETS
  hud_node
  hud_tester
  DESTINATION lib/${PROJECT_NAME}
)



ament_package()
