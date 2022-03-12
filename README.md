# ED GUI server

[![CI](https://github.com/tue-robotics/ed_gui_server/actions/workflows/main.yml/badge.svg)](https://github.com/tue-robotics/ed_gui_server/actions/workflows/main.yml)

## Installation

Requirements:

* ED (<https://github.com/tue-robotics/ed.git>)

You will also need the following system dependencies:

* geometry_msgs
* tf2
* tf2_geometry_msgs
* tf2_ros
* urdf

Check out the following packages in your workspace:

```bash
cd <your_catkin_workspace>/src
git clone https://github.com/tue-robotics/geolib2.git
git clone https://github.com/tue-robotics/ed.git
git clone https://github.com/tue-robotics/ed_gui_server.git
```

And compile

```bash
cd <your_catkin_workspace>
catkin build
```

## ROS API

 When you've started ED you should be able to listen to

```bash
rostopic echo /ed/gui/entities
```

Notice that 'rostopic echo /ed/gui/entities' will give you a constant stream of info of all entities, including their id. To get the image of a certain entity, use the service

```bash
rosservice call /ed/gui/get_entity_info
```

The service is defined here:

```bash
rossrv show ed_gui_server_msgs/GetEntityInfo
```

To store entity info (measurements needed for training perception) on the computer which runs ED, you'll have to use the service:

```text
rosservice call /ed/gui/interact "REQUEST"
```

The service is defnied here:

```bash
rossrv show ed_gui_server_msgs/Interact
```

And put the following in the `command_yaml` field: `{action: store, id: <PUT-ENTITY-ID-HERE>, type: <PUT-ENTITY-TYPE-HERE>}`

There are also services to query for entities in a specific region. For example, the following service call returns the info of entities (their id and pose) in the 20 x 20 x 20 m area around (0, 0, 0):

```bash
rosservice call /ed/gui/query_entities "{ area_min: {x: -10, y: -10, z: -10}, area_max: {x: 10, y: 10, z: 10} }"
```

Once you've received the entity ids, these can be used to query for their meshes. For example, to get the mesh of entitiy 'floor':

```bash
rosservice call /ed/gui/query_meshes '{ entity_ids: [ "floor" ] }'
```

Per entity, You'll receive a list of vertices and a list of triangles referring to the vertices. For more info on the exact format check the message describtion:

```bash
rosmsg show ed_gui_server_msgs/Mesh
```

## Tools for visualizing the state of ED

### ED Rviz Markerarray publisher

```bash
rosrun ed_gui_server ed_rviz_publisher
```

Now start RViz, and listen to the Marker topic '/ed/rviz'. You should see two blocks appearing: the blocks you specified in the configuration file. Or use the rviz_plugin from the ed_rviz_plugins package to visualize.

### ED RVIZ PLUGIN Worldmodel Display

Located in the [`ed_rviz_plugins`](https://github.com/tue-robotics/ed_rviz_plugins.git) package:

Start rviz and add the `ed_rviz_plugins/WorldMode`l display. Configure the service for querying the meshes and the ED entities topic, e.g. `/ed/gui/entities` and `/ed/gui/query_meshes`.

## Tutorial

All ED tutorials can be found in the ed_tutorials package: <https://github.com/tue-robotics/ed_tutorials>
