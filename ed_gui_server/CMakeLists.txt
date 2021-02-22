cmake_minimum_required(VERSION 3.0.2)
project(ed_gui_server)

# Compile as C++14, supported in ROS Melodic and newer
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  ed
  ed_gui_server_msgs
  geolib2
  geometry_msgs
  message_generation
  tf
  tue_config
  urdf
  visualization_msgs
)

find_package(Boost REQUIRED COMPONENTS filesystem)

catkin_package(
  CATKIN_DEPENDS ed
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(ed_gui_server_plugin
  src/gui_server_plugin.cpp
  src/robot.cpp
)
target_link_libraries(ed_gui_server_plugin ${catkin_LIBRARIES})
add_dependencies(ed_gui_server_plugin ${catkin_EXPORTED_TARGETS})

add_executable(ed_rviz_publisher tools/rviz_publisher.cpp)
target_link_libraries(ed_rviz_publisher ${catkin_LIBRARIES})
add_dependencies(ed_rviz_publisher ${catkin_EXPORTED_TARGETS})
