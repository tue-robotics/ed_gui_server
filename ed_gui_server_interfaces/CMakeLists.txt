cmake_minimum_required(VERSION 3.8)
project(ed_gui_server_interfaces)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Color.msg"
  "msg/EntityInfo.msg"
  "msg/EntityInfos.msg"
  "msg/EntityMeshAndVolumes.msg"
  "msg/Mesh.msg"
  "msg/Polygon.msg"
  "msg/Volume.msg"
  "srv/GetEntityInfo.srv"
  "srv/Interact.srv"
  "srv/Map.srv"
  "srv/QueryEntities.srv"
  "srv/QueryMeshes.srv"
  DEPENDENCIES
    geometry_msgs
    sensor_msgs
    std_msgs
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)
  ament_lint_cmake(MAX_LINE_LENGTH 120)
  ament_xmllint(MAX_LINE_LENGTH 120)
endif()

ament_package()
