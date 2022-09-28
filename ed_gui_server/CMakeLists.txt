cmake_minimum_required(VERSION 3.0.2)
project(ed_gui_server)

add_compile_options(-Wall -Werror=all)
add_compile_options(-Wextra -Werror=extra)

find_package(catkin REQUIRED COMPONENTS
  ed
  ed_gui_server_msgs
  geolib2
  geometry_msgs
  rgbd
  tf2
  tf2_geometry_msgs
  tf2_ros
  tue_config
  urdf
  visualization_msgs
)

find_package(Boost REQUIRED COMPONENTS filesystem)

catkin_package(
  LIBRARIES ed_gui_server_plugin
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
