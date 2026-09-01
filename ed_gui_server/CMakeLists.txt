cmake_minimum_required(VERSION 3.8)
project(ed_gui_server)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Werror=all)
  add_compile_options(-Wextra -Werror=extra)
endif()

find_package(ament_cmake REQUIRED)

find_package(ament_index_cpp REQUIRED)
find_package(ed REQUIRED)
find_package(ed_gui_server_interfaces REQUIRED)
find_package(geolib2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rgbd REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tue_config REQUIRED)
find_package(urdf REQUIRED)
find_package(visualization_msgs REQUIRED)

# Non-ROS dependencies
find_package(OpenCV REQUIRED)

# ------------------------------------------------------------------------------------------------
#                                            PLUGIN
# ------------------------------------------------------------------------------------------------

add_library(${PROJECT_NAME}_plugin SHARED
  src/gui_server_plugin.cpp
  src/gui_server_plugin.h
  src/robot.cpp
  src/robot.h
)
target_link_libraries(${PROJECT_NAME}_plugin PUBLIC
  ed::ed_core
  ed::ed_io
  geolib2::geolib
  pluginlib::pluginlib
  rclcpp::rclcpp
  rgbd::rgbd
  tf2::tf2
  tf2_ros::tf2_ros
  tue_config::tue_config
  urdf::urdf
  ${ed_gui_server_interfaces_TARGETS}
  ${geometry_msgs_TARGETS}
)
# Only the URDF mesh loading and the map rendering include these; keep them off the link interface.
target_link_libraries(${PROJECT_NAME}_plugin PRIVATE
  ament_index_cpp::ament_index_cpp
  ${OpenCV_LIBRARIES}
)

pluginlib_export_plugin_description_file(ed plugins.xml)

# ------------------------------------------------------------------------------------------------
#                                            TOOLS
# ------------------------------------------------------------------------------------------------

add_executable(ed_rviz_publisher tools/rviz_publisher.cpp)
target_link_libraries(ed_rviz_publisher PRIVATE
  rclcpp::rclcpp
  ${ed_gui_server_interfaces_TARGETS}
  ${geometry_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
)

# ------------------------------------------------------------------------------------------------
#                                            INSTALL
# ------------------------------------------------------------------------------------------------

install(
  TARGETS
    ed_rviz_publisher
    ${PROJECT_NAME}_plugin
  DESTINATION lib/${PROJECT_NAME}
)

# ------------------------------------------------------------------------------------------------
#                                             TESTS
# ------------------------------------------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_cmake_clang_format REQUIRED)
  find_package(ament_cmake_clang_tidy REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)
  find_package(tue_lint_config REQUIRED)

  ament_clang_format(CONFIG_FILE ${tue_lint_config_DIR}/../config/.clang-format --clang-format-version=21)
  ament_clang_tidy(CONFIG_FILE ${tue_lint_config_DIR}/../config/.clang-tidy --clang-tidy-version=21
    ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_BINARY_DIR}/compile_commands.json)
  ament_lint_cmake(MAX_LINE_LENGTH 120)
  ament_xmllint(MAX_LINE_LENGTH 120)
endif()

ament_package()
