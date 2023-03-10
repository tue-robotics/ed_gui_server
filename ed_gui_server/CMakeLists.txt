cmake_minimum_required(VERSION 3.0.2)
project(ed_gui_server)

add_compile_options(-Wall -Werror=all)
add_compile_options(-Wextra -Werror=extra)

find_package(catkin REQUIRED COMPONENTS
  ed
  ${PROJECT_NAME}_msgs
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
  LIBRARIES ${PROJECT_NAME}_plugin
)

include_directories(
  SYSTEM
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_plugin
  src/gui_server_plugin.cpp
  src/robot.cpp
)
target_link_libraries(${PROJECT_NAME}_plugin ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_plugin ${catkin_EXPORTED_TARGETS})

add_executable(ed_rviz_publisher tools/rviz_publisher.cpp)
target_link_libraries(ed_rviz_publisher ${catkin_LIBRARIES})
add_dependencies(ed_rviz_publisher ${catkin_EXPORTED_TARGETS})

install(
  FILES plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  TARGETS
    ${PROJECT_NAME}_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  TARGETS
    ed_rviz_publisher
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(catkin_lint_cmake REQUIRED)
  catkin_add_catkin_lint_test("-W2")
endif()
