#ifndef ED_GUI_SERVER_PLUGIN_H_
#define ED_GUI_SERVER_PLUGIN_H_

#include <ed/plugin.h>
#include <ed/types.h>

#include <ed_gui_server_interfaces/msg/entity_infos.hpp>
#include <ed_gui_server_interfaces/srv/get_entity_info.hpp>
#include <ed_gui_server_interfaces/srv/interact.hpp>
#include <ed_gui_server_interfaces/srv/map.hpp>
#include <ed_gui_server_interfaces/srv/query_entities.hpp>
#include <ed_gui_server_interfaces/srv/query_meshes.hpp>

#include <geolib/Shape.h>

#include <rclcpp/callback_group.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/service.hpp>

#include "robot.h"

#include <memory>
#include <string>

class GUIServerPlugin : public ed::Plugin
{

public:
    GUIServerPlugin();

    ~GUIServerPlugin() override;

    void initialize(ed::InitData& init) override;

    void process(const ed::WorldModel& world, ed::UpdateRequest& req) override;

private:
    //

    const ed::WorldModel* world_model_{nullptr};

    rclcpp::CallbackGroup::SharedPtr cb_group_;

    rclcpp::executors::SingleThreadedExecutor executor_;

    rclcpp::Publisher<ed_gui_server_interfaces::msg::EntityInfos>::SharedPtr pub_entities_;

    rclcpp::Service<ed_gui_server_interfaces::srv::QueryMeshes>::SharedPtr srv_query_meshes_;

    ed::shared_ptr<gui::Robot> robot_;

    geo::Shape person_shape_;

    void srvQueryMeshes(const std::shared_ptr<ed_gui_server_interfaces::srv::QueryMeshes::Request>& ros_req,
                        const std::shared_ptr<ed_gui_server_interfaces::srv::QueryMeshes::Response>& ros_res);

    rclcpp::Service<ed_gui_server_interfaces::srv::QueryEntities>::SharedPtr srv_query_entities_;

    void srvQueryEntities(const std::shared_ptr<ed_gui_server_interfaces::srv::QueryEntities::Request>& ros_req,
                          const std::shared_ptr<ed_gui_server_interfaces::srv::QueryEntities::Response>& ros_res);

    rclcpp::Service<ed_gui_server_interfaces::srv::GetEntityInfo>::SharedPtr srv_get_entity_info_;

    void srvGetEntityInfo(const std::shared_ptr<ed_gui_server_interfaces::srv::GetEntityInfo::Request>& ros_req,
                          const std::shared_ptr<ed_gui_server_interfaces::srv::GetEntityInfo::Response>& ros_res);

    rclcpp::Service<ed_gui_server_interfaces::srv::Interact>::SharedPtr srv_interact_;

    void srvInteract(const std::shared_ptr<ed_gui_server_interfaces::srv::Interact::Request>& ros_req,
                     const std::shared_ptr<ed_gui_server_interfaces::srv::Interact::Response>& ros_res);

    void storeMeasurement(const std::string& id, const std::string& type);

    rclcpp::Service<ed_gui_server_interfaces::srv::Map>::SharedPtr srv_map_;

    /**
     * @brief Generate a map based on the entities that need to be in-view
     * @param req Service request
     * @param res Service response
     */
    void srvMap(const std::shared_ptr<ed_gui_server_interfaces::srv::Map::Request>& req,
                const std::shared_ptr<ed_gui_server_interfaces::srv::Map::Response>& res);
};

#endif
