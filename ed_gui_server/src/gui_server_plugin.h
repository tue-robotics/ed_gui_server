#ifndef ED_GUI_SERVER_PLUGIN_H_
#define ED_GUI_SERVER_PLUGIN_H_

#include <ed/plugin.h>

#include <ed_gui_server_msgs/QueryEntities.h>
#include <ed_gui_server_msgs/QueryMeshes.h>
#include <ed_gui_server_msgs/Interact.h>
#include <ed_gui_server_msgs/GetEntityInfo.h>
#include <ed_gui_server_msgs/Map.h>

#include <geolib/Shape.h>

#include <ros/callback_queue.h>
#include <ros/service_server.h>
#include <ros/publisher.h>

#include "robot.h"

class GUIServerPlugin : public ed::Plugin
{

public:

    GUIServerPlugin();

    virtual ~GUIServerPlugin();

    void initialize(ed::InitData& init);

    void process(const ed::WorldModel& world, ed::UpdateRequest& req);

private:

    //

    const ed::WorldModel* world_model_;

    ros::CallbackQueue cb_queue_;

    ros::Publisher pub_entities_;

    ros::ServiceServer srv_query_meshes_;

    ed::shared_ptr<gui::Robot> robot_;

    geo::Shape person_shape_;

    bool srvQueryMeshes(const ed_gui_server_msgs::QueryMeshes::Request& ros_req,
                         ed_gui_server_msgs::QueryMeshes::Response& ros_res);

    ros::ServiceServer srv_query_entities_;

    bool srvQueryEntities(const ed_gui_server_msgs::QueryEntities::Request& ros_req,
                          ed_gui_server_msgs::QueryEntities::Response& ros_res);

    ros::ServiceServer srv_get_entity_info_;

    bool srvGetEntityInfo(const ed_gui_server_msgs::GetEntityInfo::Request& ros_req,
                          ed_gui_server_msgs::GetEntityInfo::Response& ros_res);

    ros::ServiceServer srv_interact_;

    bool srvInteract(const ed_gui_server_msgs::Interact::Request& ros_req,
                          ed_gui_server_msgs::Interact::Response& ros_res);

    void storeMeasurement(const std::string& id, const std::string& type);


    void entityToMsg(const ed::EntityConstPtr& e, ed_gui_server_msgs::EntityInfo& msg);

    ros::ServiceServer srv_map_;

    /**
     * @brief Generate a map based on the entities that need to be in-view
     * @param req Service request
     * @param rep Service response
     * @return success
     */
    bool srvMap(const ed_gui_server_msgs::Map::Request& req,
                ed_gui_server_msgs::Map::Response& rep);

};

#endif
