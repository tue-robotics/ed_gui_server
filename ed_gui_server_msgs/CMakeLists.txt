cmake_minimum_required(VERSION 3.0.2)
project(ed_gui_server_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  sensor_msgs
)

add_message_files(
  FILES
    Color.msg
    EntityInfo.msg
    EntityInfos.msg
    Mesh.msg
    EntityMeshAndVolumes.msg
    Polygon.msg
    Volume.msg
)

add_service_files(
  FILES
    GetEntityInfo.srv
    Interact.srv
    Map.srv
    QueryEntities.srv
    QueryMeshes.srv
)

generate_messages(
  DEPENDENCIES
    geometry_msgs
    sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs message_runtime sensor_msgs
)
