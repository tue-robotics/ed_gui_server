#include <ed_gui_server/EntityInfos.h>
#include <ed_gui_server/QueryMeshes.h>

#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>

#include <geolib/ros/msg_conversions.h>

#include <visualization_msgs/MarkerArray.h>

struct EntityViz
{
    EntityViz() : mesh_revision(0) {}

    unsigned int mesh_revision;
    visualization_msgs::Marker marker;
    unsigned int num_id;

};

std::map<std::string, EntityViz> entities;
ed_gui_server::QueryMeshes query_meshes_srv;

visualization_msgs::MarkerArray marker_msg;

// ----------------------------------------------------------------------------------------------------

float COLORS[27][3] = { { 0.6, 0.6, 0.6},
                        { 0.6, 0.6, 0.4},
                        { 0.6, 0.6, 0.2},
                        { 0.6, 0.4, 0.6},
                        { 0.6, 0.4, 0.4},
                        { 0.6, 0.4, 0.2},
                        { 0.6, 0.2, 0.6},
                        { 0.6, 0.2, 0.4},
                        { 0.6, 0.2, 0.2},
                        { 0.4, 0.6, 0.6},
                        { 0.4, 0.6, 0.4},
                        { 0.4, 0.6, 0.2},
                        { 0.4, 0.4, 0.6},
                        { 0.4, 0.4, 0.4},
                        { 0.4, 0.4, 0.2},
                        { 0.4, 0.2, 0.6},
                        { 0.4, 0.2, 0.4},
                        { 0.4, 0.2, 0.2},
                        { 0.2, 0.6, 0.6},
                        { 0.2, 0.6, 0.4},
                        { 0.2, 0.6, 0.2},
                        { 0.2, 0.4, 0.6},
                        { 0.2, 0.4, 0.4},
                        { 0.2, 0.4, 0.2},
                        { 0.2, 0.2, 0.6},
                        { 0.2, 0.2, 0.4},
                        { 0.2, 0.2, 0.2}
                      };

// ----------------------------------------------------------------------------------------------------

unsigned int djb2(const std::string& str)
{
    int hash = 5381;
    for(unsigned int i = 0; i < str.size(); ++i)
        hash = ((hash << 5) + hash) + str[i]; /* hash * 33 + c */

    if (hash < 0)
        hash = -hash;

    return hash;
}

// ----------------------------------------------------------------------------------------------------

void entityCallback(const ed_gui_server::EntityInfos::ConstPtr& msg)
{
    for(unsigned int i = 0; i < msg->entities.size(); ++i)
    {
        const ed_gui_server::EntityInfo& info = msg->entities[i];

        if (info.id.size() >= 5 && info.id.substr(info.id.size() - 5) == "floor")
            continue; // Filter floor;

        EntityViz* entity_viz;

        std::map<std::string, EntityViz>::iterator it = entities.find(info.id);
        if (it == entities.end())
        {
            entity_viz = &entities[info.id];
            entity_viz->num_id = entities.size() - 1;
        }
        else
            entity_viz = &it->second;

        if (info.mesh_revision > entity_viz->mesh_revision)
        {
            query_meshes_srv.request.entity_ids.push_back(info.id);
        }
        else if (info.mesh_revision > 0)
        {
            marker_msg.markers.push_back(entity_viz->marker);

            // Set the pose
            visualization_msgs::Marker& m = marker_msg.markers.back();
            m.pose = info.pose;
            m.header.stamp = ros::Time::now();

            if (info.color.a != 0)
            {
                m.color.r = (float)info.color.r / 255;
                m.color.g = (float)info.color.g / 255;
                m.color.b = (float)info.color.b / 255;
            }
        }
    }
}

// ----------------------------------------------------------------------------------------------------

void deserializeMesh(const std::string& id, const ed_gui_server::Mesh& mesh)
{
    std::map<std::string, EntityViz>::iterator it = entities.find(id);
    if (it == entities.end())
        return;

    EntityViz& entity_viz = it->second;
    visualization_msgs::Marker& m = entity_viz.marker;

    m.type = visualization_msgs::Marker::TRIANGLE_LIST;
    m.scale.x = m.scale.y = m.scale.z = 1.0;
    m.color.a = 1;
    m.id = entity_viz.num_id;
    m.lifetime = ros::Duration(0.2);
    m.action = visualization_msgs::Marker::ADD;
    m.header.frame_id = "/map";

    int i_color = djb2(id) % 27;
    m.color.r = COLORS[i_color][0];
    m.color.g = COLORS[i_color][1];
    m.color.b = COLORS[i_color][2];

    m.points.resize(mesh.vertices.size() / 3);
    for(unsigned int i = 0; i < m.points.size(); ++i )
    {
        unsigned int i3 = 3 * i;
        m.points[i].x = mesh.vertices[i3];
        m.points[i].y = mesh.vertices[i3 + 1];
        m.points[i].z = mesh.vertices[i3 + 2];
    }

    entity_viz.mesh_revision = mesh.revision;
}

// ----------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ed_rviz_publisher");

    std::string pub_topic = "/ed/rviz";
    if (argc > 1)
        pub_topic = argv[1];

    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/ed/gui/entities", 1, entityCallback);

    ros::ServiceClient client = nh.serviceClient<ed_gui_server::QueryMeshes>("/ed/gui/query_meshes");

    ros::Publisher pub = nh.advertise<visualization_msgs::MarkerArray>(pub_topic, 1);

    ros::Rate r(10);
    while(ros::ok())
    {
        query_meshes_srv.request.entity_ids.clear();
        marker_msg.markers.clear();

        ros::spinOnce();

        if (!marker_msg.markers.empty())
            pub.publish(marker_msg);

        // Query missing meshes (if needed)
        if (!query_meshes_srv.request.entity_ids.empty())
        {
            if (client.call(query_meshes_srv))
            {

                for(unsigned int i = 0; i < query_meshes_srv.response.meshes.size(); ++i)
                {
                    deserializeMesh(query_meshes_srv.response.entity_ids[i],
                                    query_meshes_srv.response.meshes[i]);
                }
            }
            else
            {
                ROS_ERROR("[ED RVIZ PUBLISHER] Could not query for meshes.");
            }
        }

        r.sleep();
    }

    return 0;
}
