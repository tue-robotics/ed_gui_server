#include "robot.h"

#include <urdf/model.h>

#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>

#include <ros/package.h>
#include <ros/console.h>

#include <geolib/Box.h>
#include <geolib/CompositeShape.h>
#include <geolib/io/import.h>
#include <geolib/ros/tf2_conversions.h>
#include <geolib/ros/msg_conversions.h>

#include <ed/error_context.h>
#include <ed/models/shape_loader.h>


// ----------------------------------------------------------------------------------------------------

geo::ShapePtr URDFGeometryToShape(const urdf::GeometrySharedPtr& geom)
{
    geo::ShapePtr shape;

    if (geom->type == urdf::Geometry::MESH)
    {
        urdf::Mesh* mesh = static_cast<urdf::Mesh*>(geom.get());
        if (!mesh)
        {
            ROS_WARN_NAMED("robot", "[gui_server] Robot model error: No mesh geometry defined");
            return shape;
        }

        std::string pkg_prefix = "package://";
        if (mesh->filename.substr(0, pkg_prefix.size()) == pkg_prefix)
        {
            std::string str = mesh->filename.substr(pkg_prefix.size());
            size_t i_slash = str.find("/");

            std::string pkg = str.substr(0, i_slash);
            std::string rel_filename = str.substr(i_slash + 1);
            std::string pkg_path = ros::package::getPath(pkg);
            std::string abs_filename = pkg_path + "/" + rel_filename;

            shape = geo::io::readMeshFile(abs_filename, mesh->scale.x);

            if (!shape)
                ROS_ERROR_STREAM_NAMED("robot", "[gui_server] Could not load mesh shape from '" << abs_filename << "'");
        }
    }
    else if (geom->type == urdf::Geometry::BOX)
    {
        urdf::Box* box = static_cast<urdf::Box*>(geom.get());
        if (!box)
        {
            ROS_WARN_NAMED("robot", "[gui_server] Robot model error: No box geometry defined");
            return shape;
        }

        double hx = box->dim.x / 2;
        double hy = box->dim.y / 2;
        double hz = box->dim.z / 2;

        shape.reset(new geo::Box(geo::Vector3(-hx, -hy, -hz), geo::Vector3(hx, hy, hz)));
    }
    else if (geom->type == urdf::Geometry::CYLINDER)
    {
        urdf::Cylinder* cyl = static_cast<urdf::Cylinder*>(geom.get());
        if (!cyl)
        {
            ROS_WARN_NAMED("robot", "[gui_server] Robot model error: No cylinder geometry defined");
            return shape;
        }

        shape.reset(new geo::Shape());
        ed::models::createCylinder(*shape, cyl->radius, cyl->length, 20);
    }
    else if (geom->type ==  urdf::Geometry::SPHERE)
    {
        urdf::Sphere* sphere = static_cast<urdf::Sphere*>(geom.get());
        if (!sphere)
        {
            ROS_WARN_NAMED("robot", "[gui_server] Robot model error: No sphere geometry defined");
            return shape;
        }

        shape.reset(new geo::Shape());
        ed::models::createSphere(*shape, sphere->radius);
    }

    return shape;
}

// ----------------------------------------------------------------------------------------------------

geo::ShapePtr LinkToVisual(const urdf::LinkSharedPtr& link)
{
    geo::CompositeShapePtr visual;

    for (urdf::VisualSharedPtr& vis : link->visual_array)
    {
        const urdf::GeometrySharedPtr& geom = vis->geometry;
        if (!geom)
        {
            ROS_WARN_STREAM_NAMED("robot", "[gui_server] Robot model error: missing geometry for visual in link: '" << link->name << "'");
            continue;
        }

        geo::Pose3D offset;
        const urdf::Pose& o = vis->origin;
        offset.t = geo::Vector3(o.position.x, o.position.y, o.position.z);
        offset.R.setRotation(geo::Quaternion(o.rotation.x, o.rotation.y, o.rotation.z, o.rotation.w));

        geo::ShapePtr subshape = URDFGeometryToShape(geom);
        if (!subshape)
            continue;

        if (!visual)
            visual.reset(new geo::CompositeShape());
        visual->addShape(*subshape, offset);
    }

    return visual;
}


namespace gui
{

// ----------------------------------------------------------------------------------------------------

Robot::Robot() : tf_buffer_(nullptr)
{
    ed::ErrorContext errc("robot::constructor");
}

// ----------------------------------------------------------------------------------------------------

Robot::Robot(const ed::TFBufferConstPtr& tf_buffer) : tf_buffer_(tf_buffer)
{
    ed::ErrorContext errc("robot::constructor(tf_buffer");
}


// ----------------------------------------------------------------------------------------------------

Robot::~Robot()
{
    ed::ErrorContext errc("robot::destructor");
}

// ----------------------------------------------------------------------------------------------------

void Robot::initialize(const std::string& name, const std::string& urdf_rosparam, const std::string& tf_prefix)
{
    ed::ErrorContext errc("robot::initialize; robot =", name_.c_str());

    name_ = name;

    urdf_rosparam_ = urdf_rosparam;

    tf_prefix_ = tf_prefix;
    if (!tf_prefix_.empty() && tf_prefix_.back() != '/')
        tf_prefix_ = tf_prefix_ + "/";

    // Load URDF model from parameter server
    urdf::Model robot_model;

    std::string urdf_xml;

    ros::NodeHandle nh;
    if (!nh.getParam(urdf_rosparam, urdf_xml))
    {
        ROS_ERROR_STREAM_NAMED("robot", "ROS parameter not set: '" << urdf_rosparam << "'.");
        return;
    }

    if (!robot_model.initString(urdf_xml))
    {
        ROS_ERROR_STREAM_NAMED("robot", "Could not load robot model for '" << name << ".");
        return;
    }

    std::vector<urdf::LinkSharedPtr > links;
    robot_model.getLinks(links);

    for (const urdf::LinkSharedPtr& link : links)
    {
        if (!link->visual_array.empty())
        {
            geo::ShapePtr shape = LinkToVisual(link);

            if (shape)
            {
                // The desired solution is to have static tf_prefixing
                // (see http://wiki.ros.org/tf2/Migration#Removal_of_support_for_tf_prefix)
                // The problem here is that the hsrb_description does not support (static) tf prefixing (in the xacro)
                // and hence not in the resulting urdf on the parameter server and not in tf,
                // while we do want this prefix in ED for easy filtering of WM entities.
                // Therefore we need a seperation between names of links and entities.
                std::string full_link_name = tf_prefix_ + link->name;

                // Don't prefix if link already starts with robot name
                std::string entity_name;
                if (link->name.length() >= name_.length() && link->name.substr(0, name_.length()) == name_)
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
                else if (link->visual->material_name == "White" || link->visual->material_name == "amigo_description/white"
                         || link->visual->material_name == "amigo_description/bottomcovers")
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
                    ROS_INFO_STREAM("[gui_server] " << entity_name << ": " << link->visual->material_name);
                    visual.color.a = 0;
                }

                visual.shape = shape;
                visual.link = full_link_name;

                shapes_[entity_name] = visual;
            }
        }
    }
}

// ----------------------------------------------------------------------------------------------------

geo::ShapeConstPtr Robot::getShape(const std::string& id) const
{
    ed::ErrorContext errc("robot::getShape; robot =", name_.c_str());
    ShapeMap::const_iterator it = shapes_.find(id);
    if (it != shapes_.end())
        return it->second.shape;
    return geo::ShapeConstPtr();
}

// ----------------------------------------------------------------------------------------------------

bool Robot::contains(const std::string& id) const
{
    ed::ErrorContext errc("robot::contains; robot =", name_.c_str());
    ShapeMap::const_iterator it = shapes_.find(id);
    if (it != shapes_.end())
        return true;
    return false;
}

// ----------------------------------------------------------------------------------------------------

void Robot::getEntities(std::vector<ed_gui_server_msgs::EntityInfo>& entities) const
{
    ed::ErrorContext errc("robot::getEntities; robot =", name_.c_str());

    for(ShapeMap::const_iterator it = shapes_.begin(); it != shapes_.end(); ++it)
    {
        ed_gui_server_msgs::EntityInfo e;
        e.id = it->first;

        try
        {
            geometry_msgs::TransformStamped ts = tf_buffer_->lookupTransform("map", it->second.link, ros::Time(0));
            tf2::Stamped<tf2::Transform> t;
            tf2::convert(ts, t);

            geo::Pose3D pose;
            geo::convert(t, pose);

            geo::convert(pose, e.pose);
            e.has_pose = true;

            e.color = it->second.color;

            e.visual_revision = 1;

            entities.push_back(e);
        }
        catch (tf2::TransformException& ex)
        {
            ROS_DEBUG_STREAM_NAMED("robot", "[gui_server] No transform from 'map' to '" << it->second.link << "': " << ex.what());
        }
    }
}

} // end namespace gui
