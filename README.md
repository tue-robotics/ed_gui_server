# ED GUI server [![Build Status](https://travis-ci.com/tue-robotics/ed_gui_server.svg?branch=master)](https://travis-ci.com/tue-robotics/ed_gui_server)
The ed_gui_server already runs out-of-the-box when you've started ED.

## Installation

Requirements:
* ED (https://github.com/tue-robotics/ed.git)

You will also need the following system dependencies (ROS Indigo, Ubuntu 14.04):

    sudo apt-get install ros-indigo-urdf

Check out the following packages in your workspace:

    cd <your_catkin_workspace>/src
    git clone https://github.com/tue-robotics/ed_gui_server.git

And compile

    cd <your_catkin_workspace>:
    catkin_make

## ROS API

 When you've started ED you should be able to listen to

    /ed/gui/entities

Notice that 'rostopic pub /ed/gui/entities' will give you a constant stream of info of all entities, including their id. To get the image of a certain entity, use the service

    /ed/gui/get_entity_info

The service is defined here:

    roscd ed_gui_server
    vim srv/GetEntityInfo.srv

To store entity info (measurements needed for training perception) on the computer which runs ED, you'll have to use the service:

    /ed/gui/interact

The service is defnied here:

    roscd ed_gui_server
    vim srv/Interact.srv

And put the following in the `command_yaml` field: `{action: store, id: <PUT-ENTITY-ID-HERE>, type: <PUT-ENTITY-TYPE-HERE>}`

There are also services to query for entities in a specific region. For example, the following service call returns the info of entities (their id and pose) in the 20 x 20 x 20 m area around (0, 0, 0):

    rosservice call /ed/gui/query_entities "{ area_min: {x: -10, y: -10, z: -10}, area_max: {x: 10, y: 10, z: 10} }"

Once you've received the entity ids, these can be used to query for their meshes. For example, to get the mesh of entitiy 'floor':

    rosservice call /ed/gui/query_meshes '{ entity_ids: [ "floor" ] }'

Per entity, You'll receive a list of vertices and a list of triangles referring to the vertices. For more info on the exact format, please see the Mesh.msg file in the /msg folder.

## Tools for visualizing the state of ED

### ED Rviz Markerarray publisher

    rosrun ed_gui_server ed_rviz_publisher

Now start RViz, and listen to the Marker topic '/ed/rviz'. You should see two blocks appearing: the blocks you specified in the configuration file. Or use the rviz_plugin from the ed_rviz_plugins package to visualize.

### ED RVIZ PLUGIN Worldmodel Display

Located in the https://github.com/tue-robotics/ed_rviz_plugins.git package:

Start rviz and add the ed_rviz_plugins/WorldModel display. Configure the service for querying the meshes and the ED entities topic, e.g. /ed/gui/entities and /ed/gui/query_meshes

## Tutorial

All ED tutorials can be found in the ed_tutorials package: https://github.com/tue-robotics/ed_tutorials
