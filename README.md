# ED GUI server

[![CI](https://github.com/tue-robotics/ed_gui_server/actions/workflows/main.yml/badge.svg)](https://github.com/tue-robotics/ed_gui_server/actions/workflows/main.yml)

## Installation

Requirements:

* ED (<https://github.com/tue-robotics/ed.git>)

You will also need the following system dependencies:

* geometry_msgs
* tf2
* tf2_ros
* urdf
* visualization_msgs

Check out the following packages in your workspace:

```bash
cd <your_colcon_workspace>/src
git clone https://github.com/tue-robotics/geolib2.git
git clone https://github.com/tue-robotics/ed.git
git clone https://github.com/tue-robotics/ed_gui_server.git
```

And compile

```bash
cd <your_colcon_workspace>
colcon build
```

## ROS API

The plugin is loaded by ED, so all names below are relative to the ED node (`/ed` by default).

 When you've started ED you should be able to listen to

```bash
ros2 topic echo /ed/gui/entities
```

Notice that 'ros2 topic echo /ed/gui/entities' will give you a constant stream of info of all entities, including their id. To get the image of a certain entity, use the service

```bash
ros2 service call /ed/gui/get_entity_info ed_gui_server_interfaces/srv/GetEntityInfo "{id: <PUT-ENTITY-ID-HERE>}"
```

The service is defined here:

```bash
ros2 interface show ed_gui_server_interfaces/srv/GetEntityInfo
```

To store entity info (measurements needed for training perception) on the computer which runs ED, you'll have to use the service:

```text
ros2 service call /ed/gui/interact ed_gui_server_interfaces/srv/Interact "REQUEST"
```

The service is defnied here:

```bash
ros2 interface show ed_gui_server_interfaces/srv/Interact
```

And put the following in the `command_yaml` field: `{action: store, id: <PUT-ENTITY-ID-HERE>, type: <PUT-ENTITY-TYPE-HERE>}`

There are also services to query for entities in a specific region. For example, the following service call returns the info of entities (their id and pose) in the 20 x 20 x 20 m area around (0, 0, 0):

```bash
ros2 service call /ed/gui/query_entities ed_gui_server_interfaces/srv/QueryEntities "{ area_min: {x: -10, y: -10, z: -10}, area_max: {x: 10, y: 10, z: 10} }"
```

Once you've received the entity ids, these can be used to query for their meshes. For example, to get the mesh of entitiy 'floor':

```bash
ros2 service call /ed/gui/query_meshes ed_gui_server_interfaces/srv/QueryMeshes "{ entity_ids: [floor], visual_requests: [true], volumes_requests: [false] }"
```

Per entity, You'll receive a list of vertices and a list of triangles referring to the vertices. For more info on the exact format check the message describtion:

```bash
ros2 interface show ed_gui_server_interfaces/msg/Mesh
```

## Tools for visualizing the state of ED

### ED Rviz Markerarray publisher

```bash
ros2 run ed_gui_server ed_rviz_publisher
```

Two parameters are available: `topic` (default `ed/rviz`), the MarkerArray topic to publish on, and
`rate` (default `10.0` Hz), the publish rate:

```bash
ros2 run ed_gui_server ed_rviz_publisher --ros-args -p topic:=ed/rviz -p rate:=10.0
```

Now start RViz, and listen to the Marker topic '/ed/rviz'. You should see two blocks appearing: the blocks you specified in the configuration file. Or use the rviz_plugin from the ed_rviz_plugins package to visualize.

### ED RVIZ PLUGIN Worldmodel Display

Located in the [`ed_rviz_plugins`](https://github.com/tue-robotics/ed_rviz_plugins.git) package:

Start rviz and add the `ed_rviz_plugins/WorldMode`l display. Configure the service for querying the meshes and the ED entities topic, e.g. `/ed/gui/entities` and `/ed/gui/query_meshes`.

## Tutorial

All ED tutorials can be found in the ed_tutorials package: <https://github.com/tue-robotics/ed_tutorials>
