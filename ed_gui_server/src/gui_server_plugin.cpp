#include "gui_server_plugin.h"

#include "robot.h"

#include <ed/convex_hull.h>
#include <ed/entity.h>
#include <ed/error_context.h>
#include <ed/init_data.h>
#include <ed/io/filesystem/write.h>
#include <ed/mask.h>
#include <ed/measurement.h>
#include <ed/models/shape_loader.h>
#include <ed/plugin.h>
#include <ed/rendering.h>
#include <ed/types.h>
#include <ed/world_model.h>

#include <ed_gui_server_interfaces/msg/entity_info.hpp>
#include <ed_gui_server_interfaces/msg/entity_infos.hpp>
#include <ed_gui_server_interfaces/msg/entity_mesh_and_volumes.hpp>
#include <ed_gui_server_interfaces/msg/mesh.hpp>
#include <ed_gui_server_interfaces/msg/volume.hpp>
#include <ed_gui_server_interfaces/srv/get_entity_info.hpp>
#include <ed_gui_server_interfaces/srv/interact.hpp>
#include <ed_gui_server_interfaces/srv/map.hpp>
#include <ed_gui_server_interfaces/srv/query_entities.hpp>
#include <ed_gui_server_interfaces/srv/query_meshes.hpp>

#include <geolib/CompositeShape.h>
#include <geolib/datatypes.h>
#include <geolib/math_types.h>
#include <geolib/Mesh.h>
#include <geolib/ros/msg_conversions.h>
#include <geolib/sensors/DepthCamera.h>
#include <geolib/Shape.h>

// IWYU pragma: begin_keep
#include <rgbd/image.h>
#include <rgbd/types.h>
// IWYU pragma: end_keep
#include <rgbd/ros/conversions.h>

#include <tue/config/configuration.h>
#include <tue/config/loaders/yaml.h>
#include <tue/config/reader.h>
#include <tue/config/types.h>

#include <rclcpp/callback_group.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>

#include <opencv2/core/hal/interface.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <numbers>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace
{

geo::CompositeShapePtr getPersonShape()
{
    ed::ErrorContext const errc("getPersonShape");
    auto composite = std::make_shared<geo::CompositeShape>();
    auto shape = std::make_shared<geo::Shape>();
    ed::models::createCylinder(*shape, 0.25, 1.4, 15);
    composite->addShape(*shape, geo::Pose3D(0, 0, 0.7));
    shape = std::make_shared<geo::Shape>();
    ed::models::createSphere(*shape, 0.25);
    composite->addShape(*shape, geo::Pose3D(0, 0, 1.525));
    return composite;
}

/**
 * @brief Update min/max bounds with a mesh
 * @param mesh Mesh to update the bounds with
 * @param pose Pose of the mesh
 * @param p_min Min boundary
 * @param p_max Max boundary
 */
void minMaxMesh(const geo::Mesh& mesh, const geo::Pose3D& pose, geo::Vec2& p_min, geo::Vec2& p_max)
{
    ed::ErrorContext const errc("minMaxMesh");
    for (const geo::Vector3& vertex : mesh.getPoints())
    {
        geo::Vector3 const p = pose * vertex;
        p_min.x = std::min(p.x, p_min.x);
        p_min.y = std::min(p.y, p_min.y);

        p_max.x = std::max(p.x, p_max.x);
        p_max.y = std::max(p.y, p_max.y);
    }
}

void shapeToMesh(const geo::ShapeConstPtr& shape, ed_gui_server_interfaces::msg::Mesh& mesh)
{
    ed::ErrorContext const errc("shapeToMesh");
    const std::vector<geo::Vector3>& vertices = shape->getMesh().getPoints();

    // Triangles
    const std::vector<geo::TriangleI>& triangles = shape->getMesh().getTriangleIs();
    size_t i = mesh.vertices.size();
    mesh.vertices.resize(i + (triangles.size() * 9));
    for (const geo::TriangleI& t : triangles)
    {
        for (unsigned int const index : {t.i1_, t.i2_, t.i3_})
        {
            const geo::Vector3& v = vertices[index];
            mesh.vertices[i++] = static_cast<float>(v.x);
            mesh.vertices[i++] = static_cast<float>(v.y);
            mesh.vertices[i++] = static_cast<float>(v.z);
        }
    }
}

enum class ImageCompressionType : std::uint8_t
{
    JPG,
    PNG
};

bool imageToBinary(const cv::Mat& image, std::vector<unsigned char>& data, ImageCompressionType compression_type)
{
    ed::ErrorContext const errc("imageToBinary");
    if (compression_type == ImageCompressionType::JPG)
    {
        // OpenCV compression settings
        std::vector<int> rgb_params;
        rgb_params.resize(3, 0);

        rgb_params[0] = cv::IMWRITE_JPEG_QUALITY;
        rgb_params[1] = 95; // default is 95

        // Compress image
        if (!cv::imencode(".jpg", image, data, rgb_params))
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("gui_server"), "[ED Gui Server] RGB image compression failed");
            return false;
        }
    }
    else if (compression_type == ImageCompressionType::PNG)
    {
        std::vector<int> params;
        params.resize(3, 0);

        params[0] = cv::IMWRITE_PNG_COMPRESSION;
        params[1] = 1;

        if (!cv::imencode(".png", image, data, params))
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("gui_server"), "[ED Gui Server] PNG image compression failed");
            return false;
        }
    }

    return true;
}

void entityToMsg(const ed::EntityConstPtr& e, ed_gui_server_interfaces::msg::EntityInfo& msg)
{
    ed::ErrorContext const errc("entityToMsg");

    msg.id = e->id().str();
    msg.type = e->type();
    msg.existence_probability = static_cast<float>(e->existenceProbability());
    msg.visual_revision = static_cast<std::uint32_t>(e->visualRevision());
    msg.volumes_revision = static_cast<std::uint32_t>(e->volumesRevision());

    if (e->hasType("person") && e->visualRevision() == 0)
        msg.visual_revision = 1;

    if (e->hasPose())
    {
        geo::convert(e->pose(), msg.pose);
        msg.has_pose = true;
    }
    else
    {
        msg.has_pose = false;
    }

    if (!e->visual())
    {
        const ed::ConvexHull& ch = e->convexHull();

        if (!ch.points.empty() && e->hasPose())
        {
            const geo::Pose3D& pose = e->pose();

            geo::convert(pose, msg.pose);
            msg.has_pose = true;

            msg.polygon.z_min = ch.z_min;
            msg.polygon.z_max = ch.z_max;

            size_t const size = ch.points.size();
            msg.polygon.xs.resize(size);
            msg.polygon.ys.resize(size);

            for (size_t i = 0; i < size; ++i)
            {
                msg.polygon.xs[i] = ch.points[i].x;
                msg.polygon.ys[i] = ch.points[i].y;
            }
        }
    }

    tue::config::Reader config(e->data());

    if (config.readGroup("color"))
    {
        double r = NAN;
        double g = NAN;
        double b = NAN;
        if (config.value("red", r) && config.value("green", g) && config.value("blue", b))
        {
            msg.color.r = static_cast<std::uint8_t>(255 * r);
            msg.color.g = static_cast<std::uint8_t>(255 * g);
            msg.color.b = static_cast<std::uint8_t>(255 * b);
            msg.color.a = 255;
        }
        config.endGroup();
    }

    if (e->hasFlag("highlighted"))
    {
        msg.color.a = 255;
        msg.color.r = 255;
        msg.color.g = 0;
        msg.color.b = 0;
    }

    if (e->hasType("person") || e->hasFlag("possible_human"))
    {
        msg.color.a = 255;
        msg.color.r = 255;
        msg.color.g = 255;
        msg.color.b = 0;
    }
}

} // namespace

// ----------------------------------------------------------------------------------------------------

GUIServerPlugin::GUIServerPlugin()
{
    ed::ErrorContext const errc("constructor");
    person_shape_ = *getPersonShape();
}

// ----------------------------------------------------------------------------------------------------

GUIServerPlugin::~GUIServerPlugin()
{
    ed::ErrorContext const errc("destructor");
}

// ----------------------------------------------------------------------------------------------------

void GUIServerPlugin::initialize(ed::InitData& init)
{
    ed::ErrorContext const errc("initialize");

    tue::Configuration& config = init.config;

    std::string robot_name;
    if (config.value("robot_name", robot_name, tue::config::OPTIONAL))
    {
        std::string urdf_rosparam;
        config.value("urdf_rosparam", urdf_rosparam);
        std::string tf_prefix;
        config.value("tf_prefix", tf_prefix, tue::config::OPTIONAL);
        robot_ = ed::make_shared<gui::Robot>(node_, tf_buffer_);
        robot_->initialize(robot_name, urdf_rosparam, tf_prefix);
    }

    cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    srv_query_entities_ = node_->create_service<ed_gui_server_interfaces::srv::QueryEntities>(
        "~/gui/query_entities",
        [this](const std::shared_ptr<ed_gui_server_interfaces::srv::QueryEntities::Request>& req,
               const std::shared_ptr<ed_gui_server_interfaces::srv::QueryEntities::Response>& res)
        { srvQueryEntities(req, res); },
        rclcpp::ServicesQoS(),
        cb_group_);

    srv_query_meshes_ = node_->create_service<ed_gui_server_interfaces::srv::QueryMeshes>(
        "~/gui/query_meshes",
        [this](const std::shared_ptr<ed_gui_server_interfaces::srv::QueryMeshes::Request>& req,
               const std::shared_ptr<ed_gui_server_interfaces::srv::QueryMeshes::Response>& res)
        { srvQueryMeshes(req, res); },
        rclcpp::ServicesQoS(),
        cb_group_);

    srv_get_entity_info_ = node_->create_service<ed_gui_server_interfaces::srv::GetEntityInfo>(
        "~/gui/get_entity_info",
        [this](const std::shared_ptr<ed_gui_server_interfaces::srv::GetEntityInfo::Request>& req,
               const std::shared_ptr<ed_gui_server_interfaces::srv::GetEntityInfo::Response>& res)
        { srvGetEntityInfo(req, res); },
        rclcpp::ServicesQoS(),
        cb_group_);

    srv_interact_ = node_->create_service<ed_gui_server_interfaces::srv::Interact>(
        "~/gui/interact",
        [this](const std::shared_ptr<ed_gui_server_interfaces::srv::Interact::Request>& req,
               const std::shared_ptr<ed_gui_server_interfaces::srv::Interact::Response>& res)
        { srvInteract(req, res); },
        rclcpp::ServicesQoS(),
        cb_group_);

    srv_map_ = node_->create_service<ed_gui_server_interfaces::srv::Map>(
        "~/gui/map",
        [this](const std::shared_ptr<ed_gui_server_interfaces::srv::Map::Request>& req,
               const std::shared_ptr<ed_gui_server_interfaces::srv::Map::Response>& res) { srvMap(req, res); },
        rclcpp::ServicesQoS(),
        cb_group_);

    executor_.add_callback_group(cb_group_, node_->get_node_base_interface());

    pub_entities_ = node_->create_publisher<ed_gui_server_interfaces::msg::EntityInfos>("~/gui/entities", 1);
}

// ----------------------------------------------------------------------------------------------------

void GUIServerPlugin::process(const ed::WorldModel& world, ed::UpdateRequest& /*req*/)
{
    ed::ErrorContext const errc("process");

    world_model_ = &world;
    executor_.spin_some();

    ed_gui_server_interfaces::msg::EntityInfos entities_msg;

    entities_msg.header.stamp = node_->now();
    entities_msg.header.frame_id = "map";

    entities_msg.entities.resize(world_model_->numEntities());

    unsigned int i = 0;
    for (const ed::EntityConstPtr& e : *world_model_)
    {
        entityToMsg(e, entities_msg.entities[i++]);
    }

    if (robot_)
        robot_->getEntities(entities_msg.entities);

    pub_entities_->publish(entities_msg);
}

// ----------------------------------------------------------------------------------------------------

void GUIServerPlugin::srvQueryEntities(
    const std::shared_ptr<ed_gui_server_interfaces::srv::QueryEntities::Request>& ros_req,
    const std::shared_ptr<ed_gui_server_interfaces::srv::QueryEntities::Response>& ros_res)
{
    ed::ErrorContext const errc("srvQueryEntities");
    for (const ed::EntityConstPtr& e : *world_model_)
    {
        if (!e->hasPose())
            continue;

        const geo::Pose3D& pose = e->pose();

        double const pos_x = pose.t.x;
        double const pos_y = pose.t.y;

        if (ros_req->area_min.x < pos_x && pos_x < ros_req->area_max.x && ros_req->area_min.y < pos_y &&
            pos_y < ros_req->area_max.y)
        {
            ed_gui_server_interfaces::msg::EntityInfo& info = ros_res->entities.emplace_back();

            info.id = e->id().str();
            info.visual_revision = static_cast<std::uint32_t>(e->visualRevision());
            info.volumes_revision = static_cast<std::uint32_t>(e->volumesRevision());
            geo::convert(pose, info.pose);
        }
    }
}

// ----------------------------------------------------------------------------------------------------

void GUIServerPlugin::srvGetEntityInfo(
    const std::shared_ptr<ed_gui_server_interfaces::srv::GetEntityInfo::Request>& ros_req,
    const std::shared_ptr<ed_gui_server_interfaces::srv::GetEntityInfo::Response>& ros_res)
{
    ed::ErrorContext const errc("srvGetEntityInfo");
    ed::EntityConstPtr const e = world_model_->getEntity(ros_req->id);
    if (!e)
        return;

    ros_res->type = e->type();

    // TODO: get affordances from entity
    ros_res->affordances.emplace_back("navigate-to");
    ros_res->affordances.emplace_back("pick-up");
    ros_res->affordances.emplace_back("place");

    if (e->hasPose())
    {
        const geo::Pose3D& pose = e->pose();
        std::stringstream ss_pose;
        ss_pose << "(" << pose.t.x << ", " << pose.t.y << ", " << pose.t.z << ")";
        ros_res->property_names.emplace_back("position");
        ros_res->property_values.push_back(ss_pose.str());
    }

    ed::MeasurementConstPtr const m = e->lastMeasurement();
    if (m)
    {
        const cv::Mat& rgb_image = m->image()->getRGBImage();
        const ed::ImageMask& image_mask = m->imageMask();

        cv::Mat rgb_image_masked(rgb_image.rows, rgb_image.cols, CV_8UC3, cv::Scalar(0, 0, 0));

        cv::Point min(rgb_image.cols, rgb_image.rows);
        cv::Point max(0, 0);
        for (ed::ImageMask::const_iterator it = image_mask.begin(rgb_image.cols); it != image_mask.end(); ++it)
        {
            const cv::Point2i p(it());
            rgb_image_masked.at<cv::Vec3b>(p) = rgb_image.at<cv::Vec3b>(p);

            min.x = std::min(min.x, p.x);
            min.y = std::min(min.y, p.y);
            max.x = std::max(max.x, p.x);
            max.y = std::max(max.y, p.y);
        }

        // The measurement image fields are uint8[]; on Rolling that is a rosidl::Buffer rather than a
        // std::vector, so encode into a vector and move it into the field.
        std::vector<unsigned char> measurement_image;
        imageToBinary(rgb_image_masked, measurement_image, ImageCompressionType::JPG);
        ros_res->measurement_image = std::move(measurement_image);

        // Add border to roi
        min.x = std::max(0, min.x - ros_req->measurement_image_border);
        min.y = std::max(0, min.y - ros_req->measurement_image_border);
        max.x = std::min(rgb_image.cols - 1, max.x + ros_req->measurement_image_border);
        max.y = std::min(rgb_image.rows - 1, max.y + ros_req->measurement_image_border);

        std::vector<unsigned char> measurement_image_unmasked;
        imageToBinary(rgb_image(cv::Rect(min, max)), measurement_image_unmasked, ImageCompressionType::JPG);
        ros_res->measurement_image_unmasked = std::move(measurement_image_unmasked);
    }
}

// ----------------------------------------------------------------------------------------------------

void GUIServerPlugin::srvQueryMeshes(
    const std::shared_ptr<ed_gui_server_interfaces::srv::QueryMeshes::Request>& ros_req,
    const std::shared_ptr<ed_gui_server_interfaces::srv::QueryMeshes::Response>& ros_res)
{
    ed::ErrorContext const errc("srvQueryMeshes");
    // Check size of vectors to prevent segmentation faults
    if (ros_req->entity_ids.size() != ros_req->visual_requests.size())
    {
        std::stringstream ss;
        ss << "Number of requested ids(" << ros_req->entity_ids.size()
           << ") does not match the number of visual request values (" << ros_req->visual_requests.size() << ")\n";
        ros_res->error_msg.append(ss.str());
        ros_res->error_msg.append("\n");
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("gui_server"), ss.str());
    }
    if (ros_req->entity_ids.size() != ros_req->volumes_requests.size())
    {
        std::stringstream ss;
        ss << "Number of requested ids(" << ros_req->entity_ids.size()
           << ") does not match the number of volumes request values (" << ros_req->volumes_requests.size() << ")";
        ros_res->error_msg.append(ss.str());
        ros_res->error_msg.append("\n");
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("gui_server"), ss.str());
    }
    if (!ros_res->error_msg.empty())
        return;

    for (size_t i = 0; i < ros_req->entity_ids.size(); ++i)
    {
        const std::string& id = ros_req->entity_ids[i];
        bool const visual_needs_update = ros_req->visual_requests[i];
        bool const volumes_needs_update = ros_req->volumes_requests[i];

        if (!visual_needs_update && !volumes_needs_update)
        {
            RCLCPP_WARN_STREAM(rclcpp::get_logger("gui_server"),
                               "Got request for enity: '" << id << "', but both visual and volumes request are false");
            continue;
        }

        ed::EntityConstPtr const e = world_model_->getEntity(id);
        bool robot_entity = false;
        if (!e)
        {
            if (robot_ && robot_->contains(id))
                robot_entity = true;

            if (!robot_entity)
            {
                ros_res->error_msg += "Unknown entity: '" + id + "'.\n";
                continue;
            }
        }

        ed_gui_server_interfaces::msg::EntityMeshAndVolumes& entity_geometry =
            ros_res->entity_geometries.emplace_back();

        if (visual_needs_update)
        {
            geo::ShapeConstPtr visual;
            std::uint64_t visual_revision = 0;
            if (robot_entity)
            {
                visual = robot_->getShape(id);
                if (visual)
                    visual_revision = 1;
                else
                    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("gui_server"),
                                        "No visual from robot entity: '" << id << "'");
            }
            else
            {
                // If entity is not part of the robot
                visual = e->visual();
                visual_revision = e->visualRevision();
            }
            if (!visual && e && e->hasType("person"))
            {
                geo::Shape shape_tr;
                geo::Transform compensate_z = geo::Transform::identity();
                compensate_z.t.z = -e->pose().t.z;
                shape_tr.setMesh(person_shape_.getMesh().getTransformed(compensate_z));
                visual = std::make_shared<const geo::Shape>(shape_tr);
                visual_revision = 1;
            }

            if (visual)
            {
                entity_geometry.id = id;
                entity_geometry.visual_revision = static_cast<std::uint32_t>(visual_revision);
                shapeToMesh(visual, entity_geometry.mesh);
            }
            else
            {
                RCLCPP_DEBUG_STREAM(rclcpp::get_logger("gui_server"),
                                    "Could not get a visual for entity: '" << id << "'");
            }
        }

        if (volumes_needs_update)
        {
            if (robot_entity)
            {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger("gui_server"),
                                    "Can't get volumes of robot entities. Requested for entity: '" << id << "'");
                continue;
            }

            entity_geometry.id = id;
            entity_geometry.volumes_revision = static_cast<std::uint32_t>(e->volumesRevision());
            for (const auto& [name, area_shape] : e->volumes())
            {
                if (area_shape)
                {
                    ed_gui_server_interfaces::msg::Volume& entity_volume = entity_geometry.volumes.emplace_back();
                    entity_volume.name = name;

                    shapeToMesh(area_shape, entity_volume.mesh);
                }
            }
        }
    }
}

// ----------------------------------------------------------------------------------------------------

void GUIServerPlugin::storeMeasurement(const std::string& id, const std::string& type)
{
    ed::ErrorContext const errc("storeMeasurement", ("id: '" + id + "', type: '" + type + "'").c_str());
    ed::EntityConstPtr const e = world_model_->getEntity(id);
    if (e)
    {
        char const* const home = std::getenv("HOME");
        if (home != nullptr)
        {
            std::filesystem::path const dir = std::filesystem::path(home) / ".ed" / "measurements" / type;
            std::filesystem::create_directories(dir);

            std::string const filename = (dir / ed::Entity::generateID().str()).string();
            ed::write(filename, *e);

            RCLCPP_INFO_STREAM(node_->get_logger(), "Writing entity info to '" << filename << "'.");
        }
    }
    else
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Entity with id " << id << " doesn't exist!");
}

// ----------------------------------------------------------------------------------------------------

void GUIServerPlugin::srvInteract(const std::shared_ptr<ed_gui_server_interfaces::srv::Interact::Request>& ros_req,
                                  const std::shared_ptr<ed_gui_server_interfaces::srv::Interact::Response>& ros_res)
{
    ed::ErrorContext const errc("srvInteract", ros_req->command_yaml.c_str());
    RCLCPP_DEBUG_STREAM(node_->get_logger(), "[ED Gui Server] Received command: " << ros_req->command_yaml);

    tue::Configuration params;
    tue::config::loadFromYAMLString(ros_req->command_yaml, params);

    std::string action;
    if (params.value("action", action))
    {
        if (action == "store")
        {
            std::string id;
            std::string type;
            if (params.value("id", id) && params.value("type", type))
                storeMeasurement(id, type);
            else
                RCLCPP_ERROR_STREAM(node_->get_logger(), "Please specify an id and a type!");
        }
    }

    if (params.hasError())
        ros_res->result_json = "{ error: \"" + params.error() + "\" }";
    else
        ros_res->result_json = "{}";
}

// ----------------------------------------------------------------------------------------------------

void GUIServerPlugin::srvMap(const std::shared_ptr<ed_gui_server_interfaces::srv::Map::Request>& req,
                             const std::shared_ptr<ed_gui_server_interfaces::srv::Map::Response>& res)
{
    ed::ErrorContext const errc("srvMap");
    geo::Vec2 p_min(1e9, 1e9);
    geo::Vec2 p_max(-1e9, -1e9);

    bool model_found = false;
    for (const std::string& model : req->entities_in_view)
    {
        ed::EntityConstPtr const e = world_model_->getEntity(model);
        if (!e || !e->hasPose())
        {
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("srvMap"), "Not taking into account entity: " << model);
            continue;
        }

        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("srvMap"), "Taking into account entity: " << model);

        if (e->visual())
        {
            minMaxMesh(e->visual()->getBoundingBox().getMesh(), e->pose(), p_min, p_max);
            model_found = true;
        }
        else if (e->hasType("room") && !e->volumes().empty())
        {
            for (const auto& v : e->volumes())
            {
                minMaxMesh(v.second->getBoundingBox().getMesh(), e->pose(), p_min, p_max);
                model_found = true;
            }
        }
        else
        {
            p_min.x = std::min(e->pose().t.x, p_min.x);
            p_min.y = std::min(e->pose().t.y, p_min.y);

            p_max.x = std::max(e->pose().t.x, p_max.x);
            p_max.y = std::max(e->pose().t.y, p_max.y);
        }
    }

    if (!model_found)
    {
        std::stringstream ss;
        ss << "[";
        for (const auto& e_id : req->entities_in_view)
        {
            ss << e_id << ", ";
        }
        ss << "]";
        RCLCPP_WARN_STREAM(rclcpp::get_logger("srvMap"),
                           "Could not find any of the following entities: "
                               << ss.str() << ". All entities will now be taken into account.");

        for (const ed::EntityConstPtr& e : *world_model_)
        {
            const std::string& id = e->id().str();

            if (e->visual() && e->hasPose() && !e->hasFlag("self") &&
                (id.size() < 5 || !id.ends_with("floor"))) // Filter ground plane
            {
                minMaxMesh(e->visual()->getBoundingBox().getMesh(), e->pose(), p_min, p_max);
            }
            else if (e->hasType("room") && !e->volumes().empty())
            {
                for (const auto& v : e->volumes())
                    minMaxMesh(v.second->getBoundingBox().getMesh(), e->pose(), p_min, p_max);
            }
        }
    }

    // Small padding around the entities in view
    p_max.x *= (p_max.x >= 0) ? 1.01 : 1 / 1.01;
    p_max.y *= (p_max.y >= 0) ? 1.01 : 1 / 1.01;
    p_min.x *= (p_min.x >= 0) ? 1 / 1.01 : 1.01;
    p_min.y *= (p_min.y >= 0) ? 1 / 1.01 : 1.01;

    geo::Vec2 range = p_max - p_min;
    geo::Vec2 const center = 0.5 * (p_max + p_min);
    double const dist = 1000; // Value doesn't influence the generated img

    geo::Pose3D cam_pose;
    cam_pose.t = center.projectTo3d();
    cam_pose.t.z = dist;
    cam_pose.R = geo::Matrix3::identity();
    if (range.y > range.x) // Rotate image to landscape if needed
    {
        cam_pose.R.setRPY(0, 0, -std::numbers::pi / 2);
        std::swap(range.x, range.y);
    }

    std::uint32_t const width =
        req->image_width != 0 ? req->image_width
                              : static_cast<std::uint32_t>(ed_gui_server_interfaces::srv::Map::Request::DEFAULT_WIDTH);
    std::uint32_t const height =
        req->image_height != 0
            ? req->image_height
            : static_cast<std::uint32_t>(ed_gui_server_interfaces::srv::Map::Request::DEFAULT_HEIGHT);
    // Pixels per meter
    double const focal_length = std::min(static_cast<double>(width) / range.x, static_cast<double>(height) / range.y);

    geo::DepthCamera const cam(width,
                               height,
                               focal_length * dist,
                               focal_length * dist,
                               (static_cast<double>(width) / 2) + 0.5,
                               (static_cast<double>(height) / 2) + 0.5,
                               0,
                               0);

    cv::Mat depth_image = cv::Mat(static_cast<int>(height), static_cast<int>(width), CV_32FC1, 0.0);
    cv::Scalar background_color;
    if (req->background == ed_gui_server_interfaces::srv::Map::Request::WHITE)
        background_color = cv::Scalar(255, 255, 255);
    else
        // Default black, but not completely black
        background_color = cv::Scalar(20, 20, 20);
    cv::Mat image = cv::Mat(static_cast<int>(height), static_cast<int>(width), CV_8UC3, background_color);

    geo::Pose3D const cam_pose_inv = cam_pose.inverse();
    ed::renderWorldModel(*world_model_, ed::ShowVolumes::NO_VOLUMES, cam, cam_pose_inv, depth_image, image, true);

    if (req->print_labels)
    {
        const cv::Scalar text_color = cv::Scalar(255, 255, 255) - background_color;
        for (const ed::EntityConstPtr& e : *world_model_)
        {
            const std::string& id = e->id().str();

            if (e->visual() && e->hasPose() && !e->hasFlag("self") &&
                (id.size() < 5 || !id.ends_with("floor"))) // Filter ground plane
            {
                geo::Vector3 center_entity = e->pose().getOrigin();
                center_entity.z = 0;
                cv::Point center_cv = cam.project3Dto2D(cam_pose_inv * center_entity);
                int tmp_baseline = 0;
                cv::Size const text_size = cv::getTextSize(id, cv::FONT_HERSHEY_COMPLEX_SMALL, 1, 1, &tmp_baseline);
                center_cv.x -= text_size.width / 2;
                center_cv.y += text_size.height / 2;
                cv::putText(image, id, center_cv, cv::FONT_HERSHEY_COMPLEX_SMALL, 1, text_color, 1);
            }
        }
    }

    rgbd::convert(image, res->map);
    res->pixels_per_meter_width = static_cast<float>(focal_length);
    res->pixels_per_meter_height = static_cast<float>(focal_length);

    // Convert from geolib to ROS convention
    geo::Mat3 rotate180 = geo::Mat3::identity();
    rotate180.setRPY(std::numbers::pi, 0, 0);
    cam_pose.R = cam_pose.R * rotate180;
    cam_pose.t.z = 0;

    geo::convert(cam_pose.R.transpose(), res->pose.pose.orientation);

    geo::Vec3 const cp_to_tl_image(
        -static_cast<double>(width) / 2 / focal_length, -static_cast<double>(height) / 2 / focal_length, 0);
    geo::Vec3 const tl_map = cam_pose * cp_to_tl_image;

    geo::convert(tl_map, res->pose.pose.position);
    res->pose.header.frame_id = "map";
    res->pose.header.stamp = node_->now();
}

ED_REGISTER_PLUGIN(GUIServerPlugin)
