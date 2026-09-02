#include "robot.h"

#include <ament_index_cpp/get_package_share_path.hpp>

#include <ed/error_context.h>
#include <ed/models/shape_loader.h>
#include <ed/types.h>

#include <ed_gui_server_interfaces/msg/entity_info.hpp>

#include <geolib/Box.h>
#include <geolib/CompositeShape.h>
#include <geolib/datatypes.h>
#include <geolib/io/import.h>
#include <geolib/math_types.h>
#include <geolib/ros/msg_conversions.h>
#include <geolib/Shape.h>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <std_msgs/msg/string.hpp>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/wait_for_message.hpp>

#include <tf2/exceptions.hpp>
#include <tf2/time.hpp>
#include <tf2_ros/buffer.hpp> // IWYU pragma: keep -- tf2_ros::Buffer must be complete for lookupTransform()

// urdf renamed its headers after jazzy; the old name is a deprecation shim there.
#if __has_include(<urdf/model.hpp>)
#include <urdf/model.hpp>
#else
#include <urdf/model.h>
#endif
#include <urdf_model/link.h>
#include <urdf_model/pose.h>
#include <urdf_model/types.h>

#include <chrono>
#include <cstddef>
#include <memory>
#include <string>
#include <utility>
#include <vector>

// ----------------------------------------------------------------------------------------------------

namespace
{

// A latched robot description is delivered as soon as discovery matches its publisher, so this
// only has to cover discovery, not the start-up of whoever publishes it.
constexpr std::chrono::seconds ROBOT_DESCRIPTION_TIMEOUT{5};

geo::ShapePtr urdfGeometryToShape(const urdf::GeometrySharedPtr& geom)
{
    geo::ShapePtr shape;

    if (geom->type == urdf::Geometry::MESH)
    {
        urdf::Mesh const* mesh = dynamic_cast<urdf::Mesh*>(geom.get());
        if (!mesh)
        {
            RCLCPP_WARN(rclcpp::get_logger("robot"), "[gui_server] Robot model error: No mesh geometry defined");
            return shape;
        }

        std::string const pkg_prefix = "package://";
        if (mesh->filename.starts_with(pkg_prefix))
        {
            std::string const str = mesh->filename.substr(pkg_prefix.size());
            size_t const i_slash = str.find('/');

            std::string const pkg = str.substr(0, i_slash);
            std::string const rel_filename = str.substr(i_slash + 1);
            std::string const pkg_path = ament_index_cpp::get_package_share_path(pkg).string();
            std::string const abs_filename = pkg_path + "/" + rel_filename;

            shape = geo::io::readMeshFile(abs_filename, mesh->scale.x);

            if (!shape)
                RCLCPP_ERROR_STREAM(rclcpp::get_logger("robot"),
                                    "[gui_server] Could not load mesh shape from '" << abs_filename << "'");
        }
    }
    else if (geom->type == urdf::Geometry::BOX)
    {
        urdf::Box const* box = dynamic_cast<urdf::Box*>(geom.get());
        if (!box)
        {
            RCLCPP_WARN(rclcpp::get_logger("robot"), "[gui_server] Robot model error: No box geometry defined");
            return shape;
        }

        double const hx = box->dim.x / 2;
        double const hy = box->dim.y / 2;
        double const hz = box->dim.z / 2;

        shape = std::make_shared<geo::Box>(geo::Vector3(-hx, -hy, -hz), geo::Vector3(hx, hy, hz));
    }
    else if (geom->type == urdf::Geometry::CYLINDER)
    {
        urdf::Cylinder const* cyl = dynamic_cast<urdf::Cylinder*>(geom.get());
        if (!cyl)
        {
            RCLCPP_WARN(rclcpp::get_logger("robot"), "[gui_server] Robot model error: No cylinder geometry defined");
            return shape;
        }

        shape = std::make_shared<geo::Shape>();
        ed::models::createCylinder(*shape, cyl->radius, cyl->length, 20);
    }
    else if (geom->type == urdf::Geometry::SPHERE)
    {
        urdf::Sphere const* sphere = dynamic_cast<urdf::Sphere*>(geom.get());
        if (!sphere)
        {
            RCLCPP_WARN(rclcpp::get_logger("robot"), "[gui_server] Robot model error: No sphere geometry defined");
            return shape;
        }

        shape = std::make_shared<geo::Shape>();
        ed::models::createSphere(*shape, sphere->radius);
    }

    return shape;
}

// ----------------------------------------------------------------------------------------------------

geo::ShapePtr linkToVisual(const urdf::LinkSharedPtr& link)
{
    geo::CompositeShapePtr visual;

    for (const urdf::VisualSharedPtr& vis : link->visual_array)
    {
        const urdf::GeometrySharedPtr& geom = vis->geometry;
        if (!geom)
        {
            RCLCPP_WARN_STREAM(rclcpp::get_logger("robot"),
                               "[gui_server] Robot model error: missing geometry for visual in link: '" << link->name
                                                                                                        << "'");
            continue;
        }

        geo::Pose3D offset;
        const urdf::Pose& o = vis->origin;
        offset.t = geo::Vector3(o.position.x, o.position.y, o.position.z);
        offset.R.setRotation(geo::Quaternion(o.rotation.x, o.rotation.y, o.rotation.z, o.rotation.w));

        geo::ShapePtr const subshape = urdfGeometryToShape(geom);
        if (!subshape)
            continue;

        if (!visual)
            visual = std::make_shared<geo::CompositeShape>();
        visual->addShape(*subshape, offset);
    }

    return visual;
}

} // namespace

namespace gui
{

// ----------------------------------------------------------------------------------------------------

Robot::Robot(rclcpp::Node::SharedPtr node, ed::TFBufferConstPtr tf_buffer) :
    node_(std::move(node)), tf_buffer_(std::move(tf_buffer))
{
    ed::ErrorContext const errc("robot::constructor");
}

// ----------------------------------------------------------------------------------------------------

Robot::~Robot()
{
    ed::ErrorContext const errc("robot::destructor");
}

// ----------------------------------------------------------------------------------------------------

bool Robot::initialize(const std::string& name,
                       const std::string& urdf_topic,
                       const std::string& tf_prefix,
                       std::string& error)
{
    ed::ErrorContext const errc("robot::initialize; robot =", name.c_str());

    name_ = name;

    tf_prefix_ = tf_prefix;
    if (!tf_prefix_.empty() && tf_prefix_.back() != '/')
        tf_prefix_ = tf_prefix_ + "/";

    // In ROS 1 the URDF was read from the global parameter server. ROS 2 has none: the robot
    // description is published latched (transient local), by robot_state_publisher. Subscribing
    // with matching durability is what makes a description published before ED started arrive.
    std_msgs::msg::String urdf_msg;
    if (!rclcpp::wait_for_message(
            urdf_msg, node_, urdf_topic, ROBOT_DESCRIPTION_TIMEOUT, rclcpp::QoS(1).transient_local().reliable()))
    {
        error = "No robot description published on '" + urdf_topic + "'.";
        return false;
    }

    return buildModel(urdf_msg.data, error);
}

// ----------------------------------------------------------------------------------------------------

bool Robot::buildModel(const std::string& urdf_xml, std::string& error)
{
    ed::ErrorContext const errc("robot::buildModel; robot =", name_.c_str());

    urdf::Model robot_model;

    if (!robot_model.initString(urdf_xml))
    {
        error = "Could not load robot model for '" + name_ + "'.";
        return false;
    }

    std::vector<urdf::LinkSharedPtr> links;
    robot_model.getLinks(links);

    for (const urdf::LinkSharedPtr& link : links)
    {
        if (link->visual_array.empty())
            continue;

        geo::ShapePtr const shape = linkToVisual(link);

        if (!shape)
            continue;

        // The desired solution is to have static tf_prefixing
        // (see http://wiki.ros.org/tf2/Migration#Removal_of_support_for_tf_prefix)
        // The problem here is that the hsrb_description does not support (static) tf prefixing (in the xacro)
        // and hence not in the resulting urdf on the parameter server and not in tf,
        // while we do want this prefix in ED for easy filtering of WM entities.
        // Therefore we need a seperation between names of links and entities.
        std::string const full_link_name = tf_prefix_ + link->name;

        // Don't prefix if link already starts with robot name
        std::string entity_name;
        if (link->name.starts_with(name_))
            entity_name = link->name;
        else
            entity_name = name_ + "/" + link->name;

        Visual visual;

        // Determine color

        visual.color.a = 255;
        if (link->visual->material_name == "Black" || full_link_name == "/amigo/base_kinect/openni_camera")
        {
            visual.color.r = 0;
            visual.color.g = 0;
            visual.color.b = 0;
        }
        else if (link->visual->material_name == "White" || link->visual->material_name == "amigo_description/white" ||
                 link->visual->material_name == "amigo_description/bottomcovers")
        {
            visual.color.r = 204;
            visual.color.g = 204;
            visual.color.b = 204;
        }
        else if (link->visual->material_name == "Grey" || link->visual->material_name == "amigo_description/aluminium")
        {
            visual.color.r = 102;
            visual.color.g = 102;
            visual.color.b = 102;
        }
        else if (link->visual->material_name == "amigo_description/orange")
        {
            visual.color.r = 204;
            visual.color.g = 102;
            visual.color.b = 0;
        }
        else if (link->visual->material_name == "amigo_description/logo")
        {
            visual.color.r = 0;
            visual.color.g = 0;
            visual.color.b = 102;
        }
        else
        {
            RCLCPP_INFO_STREAM(node_->get_logger(),
                               "[gui_server] " << entity_name << ": " << link->visual->material_name);
            visual.color.a = 0;
        }

        visual.shape = shape;
        visual.link = full_link_name;

        shapes_[entity_name] = visual;
    }

    return true;
}

// ----------------------------------------------------------------------------------------------------

geo::ShapeConstPtr Robot::getShape(const std::string& id) const
{
    ed::ErrorContext const errc("robot::getShape; robot =", name_.c_str());
    auto const it = shapes_.find(id);
    if (it != shapes_.end())
        return it->second.shape;
    return {};
}

// ----------------------------------------------------------------------------------------------------

bool Robot::contains(const std::string& id) const
{
    ed::ErrorContext const errc("robot::contains; robot =", name_.c_str());
    return shapes_.contains(id);
}

// ----------------------------------------------------------------------------------------------------

void Robot::getEntities(std::vector<ed_gui_server_interfaces::msg::EntityInfo>& entities) const
{
    ed::ErrorContext const errc("robot::getEntities; robot =", name_.c_str());

    for (const auto& [entity_name, visual] : shapes_)
    {
        ed_gui_server_interfaces::msg::EntityInfo e;
        e.id = entity_name;

        try
        {
            geometry_msgs::msg::TransformStamped const ts =
                tf_buffer_->lookupTransform("map", visual.link, tf2::TimePointZero);

            geo::Pose3D pose;
            geo::convert(ts.transform, pose);

            geo::convert(pose, e.pose);
            e.has_pose = true;

            e.color = visual.color;

            e.visual_revision = 1;

            entities.push_back(e);
        }
        catch (const tf2::TransformException& ex)
        {
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("robot"),
                                "[gui_server] No transform from 'map' to '" << visual.link << "': " << ex.what());
        }
    }
}

} // namespace gui
