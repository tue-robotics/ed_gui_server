#include <ed_gui_server_interfaces/msg/entity_info.hpp>
#include <ed_gui_server_interfaces/msg/entity_infos.hpp>
#include <ed_gui_server_interfaces/msg/entity_mesh_and_volumes.hpp>
#include <ed_gui_server_interfaces/msg/mesh.hpp>
#include <ed_gui_server_interfaces/srv/query_meshes.hpp>

#include <geometry_msgs/msg/point.hpp>

#include <rclcpp/callback_group.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/future_return_code.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/utilities.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace
{

struct EntityViz
{
    std::uint32_t visual_revision{0};
    visualization_msgs::msg::Marker marker;
    visualization_msgs::msg::Marker text_marker;
    std::int32_t num_id{0};
    std::int32_t text_num_id{0};
};

// ----------------------------------------------------------------------------------------------------

constexpr std::array<std::array<float, 3>, 27> COLORS = {
    {{0.6, 0.6, 0.6}, {0.6, 0.6, 0.4}, {0.6, 0.6, 0.2}, {0.6, 0.4, 0.6}, {0.6, 0.4, 0.4}, {0.6, 0.4, 0.2},
     {0.6, 0.2, 0.6}, {0.6, 0.2, 0.4}, {0.6, 0.2, 0.2}, {0.4, 0.6, 0.6}, {0.4, 0.6, 0.4}, {0.4, 0.6, 0.2},
     {0.4, 0.4, 0.6}, {0.4, 0.4, 0.4}, {0.4, 0.4, 0.2}, {0.4, 0.2, 0.6}, {0.4, 0.2, 0.4}, {0.4, 0.2, 0.2},
     {0.2, 0.6, 0.6}, {0.2, 0.6, 0.4}, {0.2, 0.6, 0.2}, {0.2, 0.4, 0.6}, {0.2, 0.4, 0.4}, {0.2, 0.4, 0.2},
     {0.2, 0.2, 0.6}, {0.2, 0.2, 0.4}, {0.2, 0.2, 0.2}}};

// ----------------------------------------------------------------------------------------------------

std::uint32_t djb2(const std::string& str)
{
    // Unsigned arithmetic: the ROS 1 version hashed into a signed int and negated on overflow,
    // which is undefined behaviour. The hash only picks a display color.
    std::uint32_t hash = 5381;
    for (char const c : str)
        hash = (hash << 5U) + hash + static_cast<std::uint32_t>(c); /* hash * 33 + c */

    return hash;
}

// ----------------------------------------------------------------------------------------------------

void meshToMarker(const ed_gui_server_interfaces::msg::Mesh& mesh, visualization_msgs::msg::Marker& m)
{
    m.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
    m.scale.x = 1.0;
    m.scale.y = 1.0;
    m.scale.z = 1.0;

    m.points.resize(mesh.vertices.size() / 3);
    for (size_t i = 0; i < m.points.size(); ++i)
    {
        size_t const i3 = 3 * i;
        m.points[i].x = mesh.vertices[i3];
        m.points[i].y = mesh.vertices[i3 + 1];
        m.points[i].z = mesh.vertices[i3 + 2];
    }
}

// ----------------------------------------------------------------------------------------------------

void polygonToMarker(const ed_gui_server_interfaces::msg::EntityInfo& e, visualization_msgs::msg::Marker& m)
{
    m.type = visualization_msgs::msg::Marker::LINE_LIST;
    m.scale.x = 0.01;

    for (size_t i = 0; i < e.polygon.xs.size(); ++i)
    {
        size_t const j = (i + 1) % e.polygon.xs.size();

        float const x1 = e.polygon.xs[i];
        float const x2 = e.polygon.xs[j];

        float const y1 = e.polygon.ys[i];
        float const y2 = e.polygon.ys[j];

        geometry_msgs::msg::Point p;

        // low line
        p.x = x1;
        p.y = y1;
        p.z = e.polygon.z_min;
        m.points.push_back(p);

        p.x = x2;
        p.y = y2;
        p.z = e.polygon.z_min;
        m.points.push_back(p);

        // high line
        p.x = x1;
        p.y = y1;
        p.z = e.polygon.z_max;
        m.points.push_back(p);

        p.x = x2;
        p.y = y2;
        p.z = e.polygon.z_max;
        m.points.push_back(p);

        // vertical line
        p.x = x1;
        p.y = y1;
        p.z = e.polygon.z_min;
        m.points.push_back(p);

        p.x = x1;
        p.y = y1;
        p.z = e.polygon.z_max;
        m.points.push_back(p);
    }
}

// ----------------------------------------------------------------------------------------------------

class RvizPublisher : public rclcpp::Node
{

public:
    RvizPublisher() : Node("ed_rviz_publisher"), rate_(declare_parameter<double>("rate", 10.0))
    {
        std::string const pub_topic = declare_parameter<std::string>("topic", "ed/rviz");

        sub_entities_ = create_subscription<ed_gui_server_interfaces::msg::EntityInfos>(
            "ed/gui/entities",
            1,
            [this](const ed_gui_server_interfaces::msg::EntityInfos::ConstSharedPtr& msg) { entityCallback(msg); });

        // Own callback group and executor: the mesh query is synchronous and may not dispatch the
        // entity subscription, which mutates the markers being published.
        cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
        client_query_meshes_ = create_client<ed_gui_server_interfaces::srv::QueryMeshes>(
            "ed/gui/query_meshes", rclcpp::ServicesQoS(), cb_group_);

        executor_.add_callback_group(cb_group_, get_node_base_interface());

        pub_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>(pub_topic, 1);
    }

    [[nodiscard]]
    double rate() const
    {
        return rate_;
    }

    /**
     * @brief Publish the markers collected since the previous call and query the meshes that are
     * still missing
     */
    void publishAndQuery()
    {
        if (!marker_msg_.markers.empty())
            pub_markers_->publish(marker_msg_);

        marker_msg_.markers.clear();

        if (missing_visuals_.empty())
            return;

        auto request = std::make_shared<ed_gui_server_interfaces::srv::QueryMeshes::Request>();
        request->entity_ids = std::move(missing_visuals_);
        missing_visuals_.clear();
        // The server rejects the whole request unless there is a flag per entity id. This publisher
        // only draws visuals, never volumes.
        request->visual_requests.assign(request->entity_ids.size(), true);
        request->volumes_requests.assign(request->entity_ids.size(), false);

        if (!client_query_meshes_->service_is_ready())
        {
            RCLCPP_WARN_THROTTLE(get_logger(),
                                 *get_clock(),
                                 5000,
                                 "[ED RVIZ PUBLISHER] Service '%s' not available.",
                                 client_query_meshes_->get_service_name());
            return;
        }

        auto future = client_query_meshes_->async_send_request(request);
        if (executor_.spin_until_future_complete(future, std::chrono::seconds(5)) != rclcpp::FutureReturnCode::SUCCESS)
        {
            client_query_meshes_->remove_pending_request(future);
            RCLCPP_ERROR(get_logger(), "[ED RVIZ PUBLISHER] Could not query for meshes.");
            return;
        }

        auto const response = future.get();
        if (!response->error_msg.empty())
            RCLCPP_ERROR_STREAM(get_logger(), "[ED RVIZ PUBLISHER] Query for meshes: " << response->error_msg);

        for (const ed_gui_server_interfaces::msg::EntityMeshAndVolumes& entity_geometry : response->entity_geometries)
        {
            auto const it = entities_.find(entity_geometry.id);
            if (it == entities_.end())
                continue;

            meshToMarker(entity_geometry.mesh, it->second.marker);

            it->second.visual_revision = entity_geometry.visual_revision;
        }
    }

private:
    void initMarker(const std::string& id, visualization_msgs::msg::Marker& m) const
    {
        m.color.a = 1;
        m.lifetime = rclcpp::Duration::from_seconds(4.0 / rate_);
        m.action = visualization_msgs::msg::Marker::ADD;
        m.header.frame_id = "map";

        std::uint32_t const i_color = djb2(id) % COLORS.size();
        m.color.r = COLORS[i_color][0];
        m.color.g = COLORS[i_color][1];
        m.color.b = COLORS[i_color][2];
    }

    void entityCallback(const ed_gui_server_interfaces::msg::EntityInfos::ConstSharedPtr& msg)
    {
        for (const ed_gui_server_interfaces::msg::EntityInfo& info : msg->entities)
        {
            if (info.id.ends_with("floor"))
                continue; // Filter floor

            if (!info.has_pose)
                continue;

            EntityViz* entity_viz = nullptr;

            auto const it = entities_.find(info.id);
            if (it == entities_.end())
            {
                entity_viz = &entities_[info.id];
                entity_viz->num_id = static_cast<std::int32_t>((entities_.size() - 1) * 2);
                entity_viz->text_num_id = entity_viz->num_id + 1;

                initMarker(info.id, entity_viz->marker);
                entity_viz->marker.id = entity_viz->num_id;

                initMarker(info.id, entity_viz->text_marker);
                entity_viz->text_marker.id = entity_viz->text_num_id * 1000;
            }
            else
                entity_viz = &it->second;

            // HACK!: Check if this is a human
            if (info.color.a == 1 && info.color.r == 2 && info.color.g == 3 && (info.color.b == 4 || info.color.b == 5))
            {
                // This is a human
                visualization_msgs::msg::Marker& m = marker_msg_.markers.emplace_back();

                initMarker(info.id, m);
                m.id = entity_viz->num_id;

                m.header.stamp = now();
                m.type = visualization_msgs::msg::Marker::CYLINDER;

                m.color.a = 1.0;

                if (info.color.b == 4)
                {
                    // human
                    m.color.r = 0.2;
                    m.color.g = 0.7;
                    m.color.b = 1.0;
                }
                else
                {
                    // possible human (based on laser)
                    m.color.r = 1.0;
                    m.color.g = 0.7;
                    m.color.b = 0.2;
                }

                m.pose = info.pose;
                m.pose.position.z = .5;

                m.scale.x = 0.3;
                m.scale.y = 0.3;
                m.scale.z = 1.0;
                m.ns = "humans";

                continue;
            }

            if (info.visual_revision > entity_viz->visual_revision)
            {
                missing_visuals_.push_back(info.id);
                continue;
            }

            marker_msg_.markers.push_back(entity_viz->marker);
            visualization_msgs::msg::Marker& m = marker_msg_.markers.back();

            // Set the pose and timestamp
            m.pose = info.pose;
            m.header.stamp = now();
            m.ns = "entities";

            // Set the color
            if (info.color.a != 0)
            {
                m.color.r = static_cast<float>(info.color.r) / 255;
                m.color.g = static_cast<float>(info.color.g) / 255;
                m.color.b = static_cast<float>(info.color.b) / 255;
            }

            m.color.a = info.existence_probability;

            if (info.visual_revision == 0)
            {
                // Update polygon
                polygonToMarker(info, m);
            }

            // Add text
            marker_msg_.markers.push_back(entity_viz->text_marker);
            visualization_msgs::msg::Marker& m_text = marker_msg_.markers.back();

            m_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;

            m_text.scale.x = 0.1;
            m_text.scale.y = 0.1;
            m_text.scale.z = 0.1;

            m_text.color.r = 0.9;
            m_text.color.g = 0.9;
            m_text.color.b = 0.9;

            m_text.pose = m.pose;
            m_text.pose.position.z += 0.1;
            m_text.header = m.header;
            m_text.ns = "labels";

            m_text.text = info.id.substr(0, 4);

            if (!info.type.empty())
                m_text.text += " (" + info.type + ")";
        }
    }

    double rate_;

    std::map<std::string, EntityViz> entities_;

    std::vector<std::string> missing_visuals_;

    visualization_msgs::msg::MarkerArray marker_msg_;

    rclcpp::CallbackGroup::SharedPtr cb_group_;
    rclcpp::executors::SingleThreadedExecutor executor_;

    rclcpp::Subscription<ed_gui_server_interfaces::msg::EntityInfos>::SharedPtr sub_entities_;
    rclcpp::Client<ed_gui_server_interfaces::srv::QueryMeshes>::SharedPtr client_query_meshes_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
};

} // namespace

// ----------------------------------------------------------------------------------------------------

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto const node = std::make_shared<RvizPublisher>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    rclcpp::WallRate rate(node->rate());
    while (rclcpp::ok())
    {
        executor.spin_some();

        node->publishAndQuery();

        rate.sleep();
    }

    rclcpp::shutdown();

    return 0;
}
