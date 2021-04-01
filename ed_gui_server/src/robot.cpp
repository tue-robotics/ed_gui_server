#include "robot.h"

#include <urdf/model.h>

#include <tf/transform_datatypes.h>

#include <ros/package.h>
#include <ros/console.h>

#include <geolib/Importer.h>
#include <geolib/ros/tf_conversions.h>
#include <geolib/ros/msg_conversions.h>
#include <geolib/Box.h>

#include <ed/error_context.h>

namespace gui
{

// ----------------------------------------------------------------------------------------------------

Robot::Robot() : tf_listener_(nullptr)
{
}

// ----------------------------------------------------------------------------------------------------

Robot::~Robot()
{
    delete tf_listener_;
}

// ----------------------------------------------------------------------------------------------------

void Robot::initialize(const std::string& name, const std::string& urdf_rosparam, const std::string& tf_prefix)
{
    ed::ErrorContext errc("Robot::initialize; robot =", name.c_str());

    name_ = name;

    urdf_rosparam_ = urdf_rosparam;

    tf_prefix_ = tf_prefix;
    if (!tf_prefix_.empty() && tf_prefix_.back() != '/')
        tf_prefix_ = tf_prefix_ + "/";

    // Initialize TF listener
    if (!tf_listener_)
        tf_listener_ = new tf::TransformListener();

    // Load URDF model from parameter server
    urdf::Model robot_model;

    std::string urdf_xml;

    ros::NodeHandle nh;
    if (!nh.getParam(urdf_rosparam, urdf_xml))
    {
        ROS_ERROR_STREAM("ROS parameter not set: '" << urdf_rosparam << "'.");
        return;
    }

    if (!robot_model.initString(urdf_xml))
    {
        ROS_ERROR_STREAM("Could not load robot model for '" << name << ".");
        return;
    }

    std::vector<urdf::LinkSharedPtr > links;
    robot_model.getLinks(links);

    for(std::vector<urdf::LinkSharedPtr >::const_iterator it = links.begin(); it != links.end(); ++it)
    {
        const urdf::LinkSharedPtr& link = *it;
        if (link->visual && link->visual->geometry)
        {
            geo::ShapePtr shape;

            geo::Pose3D offset;
            const urdf::Pose o = link->visual->origin;
            offset.t = geo::Vector3(o.position.x, o.position.y, o.position.z);
            offset.R.setRotation(geo::Quaternion(o.rotation.x, o.rotation.y, o.rotation.z, o.rotation.w));

//            std::cout << link->name << ": " << offset << std::endl;
//            std::cout << "    " << o.rotation.x << ", " << o.rotation.y<< ", " << o.rotation.z<< ", " << o.rotation.w << std::endl;

            if (link->visual->geometry->type == urdf::Geometry::MESH)
            {
                urdf::Mesh* mesh = static_cast<urdf::Mesh*>(link->visual->geometry.get());
                if (mesh)
                {
                    std::string pkg_prefix = "package://";
                    if (mesh->filename.substr(0, pkg_prefix.size()) == pkg_prefix)
                    {
                        std::string str = mesh->filename.substr(pkg_prefix.size());
                        size_t i_slash = str.find("/");

                        std::string pkg = str.substr(0, i_slash);
                        std::string rel_filename = str.substr(i_slash + 1);
                        std::string pkg_path = ros::package::getPath(pkg);
                        std::string abs_filename = pkg_path + "/" + rel_filename;

                        geo::Importer importer;
                        shape = importer.readMeshFile(abs_filename, mesh->scale.x);
                        if (!shape)
                            ROS_ERROR_STREAM("[ed_gui_server] Couldn't load shape for link: " << link->name);
                    }
                }
            }
            else if (link->visual->geometry->type == urdf::Geometry::BOX)
            {
                urdf::Box* box = static_cast<urdf::Box*>(link->visual->geometry.get());
                if (box)
                {
                    double hx = box->dim.x / 2;
                    double hy = box->dim.y / 2;
                    double hz = box->dim.z / 2;

                    shape.reset(new geo::Box(geo::Vector3(-hx, -hy, -hz), geo::Vector3(hx, hy, hz)));
                }
            }
            else if (link->visual->geometry->type == urdf::Geometry::CYLINDER)
            {
                urdf::Cylinder* cyl = static_cast<urdf::Cylinder*>(link->visual->geometry.get());
                if (cyl)
                {
                    geo::Mesh mesh;

                    int N = 20;

                    // Calculate vertices
                    for(int i = 0; i < N; ++i)
                    {
                        double a = 6.283 * i / N;
                        double x = sin(a) * cyl->radius;
                        double y = cos(a) * cyl->radius;

                        mesh.addPoint(x, y, -cyl->length / 2);
                        mesh.addPoint(x, y, cyl->length / 2);
                    }

                    // Calculate triangles
                    for(int i = 1; i < N - 1; ++i)
                    {
                        int i2 = 2 * i;
                        mesh.addTriangle(0, i2, i2 + 2);
                        mesh.addTriangle(1, i2 + 1, i2 + 3);
                    }

                    for(int i = 0; i < N; ++i)
                    {
                        int j = (i + 1) % N;
                        mesh.addTriangle(i * 2, j * 2, i * 2 + 1);
                        mesh.addTriangle(i * 2 + 1, j * 2, j * 2 + 1);
                    }

                    shape.reset(new geo::Shape());
                    shape->setMesh(mesh);
                }
            }

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
                if (link->name.substr(0, name_.length()) == name_)
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
                    ROS_INFO_STREAM("[ed_gui_server] " << entity_name << ": " << link->visual->material_name);
                    visual.color.a = 0;
                }

                visual.offset = offset;
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
    ShapeMap::const_iterator it = shapes_.find(id);
    if (it != shapes_.end())
        return it->second.shape;
    return geo::ShapeConstPtr();
}

// ----------------------------------------------------------------------------------------------------

void Robot::getEntities(std::vector<ed_gui_server_msgs::EntityInfo>& entities) const
{
    ed::ErrorContext errc("Robot::getEntities; robot =", name_.c_str());

    for(ShapeMap::const_iterator it = shapes_.begin(); it != shapes_.end(); ++it)
    {
        ed_gui_server_msgs::EntityInfo e;
        e.id = it->first;

        try
        {
            tf::StampedTransform t;
            tf_listener_->lookupTransform("/map", it->second.link, ros::Time(0), t);

            geo::Pose3D pose;
            geo::convert(t, pose);

            // correct for mesh offset
            pose = pose * it->second.offset;

            geo::convert(pose, e.pose);
            e.has_pose = true;

            e.color = it->second.color;

            entities.push_back(e);
        }
        catch (tf::TransformException& ex)
        {
//            ROS_ERROR_STREAM("[ed_gui_server] No transform from '/map' to '" << it->second.link << "': " << ex.what());
        }
    }
}

} // end namespace gui

