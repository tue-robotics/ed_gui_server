#include "gui_server_plugin.h"

#include <ros/node_handle.h>
#include <ros/advertise_service_options.h>

#include <ed/world_model.h>
#include <ed/entity.h>
#include <ed/measurement.h>
#include <ed/world_model/transform_crawler.h>

#include <rgbd/Image.h>

#include <geolib/Shape.h>
#include <geolib/ros/msg_conversions.h>

#include <tue/config/configuration.h>
#include <tue/config/loaders/yaml.h>
#include <tue/config/reader.h>

#include <ed_gui_server/EntityInfos.h>

#include <ed/io/filesystem/write.h>

#include <boost/filesystem.hpp>

#include <opencv2/highgui/highgui.hpp>

void entityToMsg(const ed::EntityConstPtr& e, const geo::Pose3D& e_pose, ed_gui_server::EntityInfo& msg)
{
    msg.id = e->id().str();
    msg.mesh_revision = e->shapeRevision();
    geo::convert(e_pose, msg.pose);

    if (!e->shape() && !e->convexHull().chull.empty())
    {
        const ed::ConvexHull2D& ch = e->convexHull();        

        msg.polygon.z_min = ch.min_z;
        msg.polygon.z_max = ch.max_z;

        unsigned int size = ch.chull.size();
        msg.polygon.xs.resize(size);
        msg.polygon.ys.resize(size);

        msg.pose.position.x = ch.center_point.x;
        msg.pose.position.y = ch.center_point.y;

        for(unsigned int i = 0; i < size; ++i)
        {
            msg.polygon.xs[i] = ch.chull[i].x - msg.pose.position.x;
            msg.polygon.ys[i] = ch.chull[i].y - msg.pose.position.y;
//            msg.polygon.xs[i] = ch.chull[i].x + ch.center_point.x - msg.pose.position.x;
//            msg.polygon.ys[i] = ch.chull[i].y + ch.center_point.y - msg.pose.position.y;
        }
    }

    tue::config::Reader config(e->data());

    if (config.readGroup("color"))
    {
        double r, g, b;
        if (config.value("red", r) && config.value("green", g) && config.value("blue", b))
        {
            msg.color.r = 255 * r;
            msg.color.g = 255 * g;
            msg.color.b = 255 * b;
            msg.color.a = 255;
        }
        config.endGroup();
    }
}

// ----------------------------------------------------------------------------------------------------

GUIServerPlugin::GUIServerPlugin()
{
}

// ----------------------------------------------------------------------------------------------------

GUIServerPlugin::~GUIServerPlugin()
{
}

// ----------------------------------------------------------------------------------------------------

void GUIServerPlugin::configure(tue::Configuration config)
{
    std::string robot_name;
    if (config.value("robot_name", robot_name, tue::OPTIONAL))
    {
        robot_.initialize(robot_name);
    }
}

// ----------------------------------------------------------------------------------------------------

void GUIServerPlugin::initialize()
{
    ros::NodeHandle nh;

    ros::AdvertiseServiceOptions opt_srv_entities =
            ros::AdvertiseServiceOptions::create<ed_gui_server::QueryEntities>(
                "/ed/gui/query_entities", boost::bind(&GUIServerPlugin::srvQueryEntities, this, _1, _2),
                ros::VoidPtr(), &cb_queue_);

    srv_query_entities_ = nh.advertiseService(opt_srv_entities);


    ros::AdvertiseServiceOptions opt_srv_meshes =
            ros::AdvertiseServiceOptions::create<ed_gui_server::QueryMeshes>(
                "/ed/gui/query_meshes", boost::bind(&GUIServerPlugin::srvQueryMeshes, this, _1, _2),
                ros::VoidPtr(), &cb_queue_);

    srv_query_meshes_ = nh.advertiseService(opt_srv_meshes);

    ros::AdvertiseServiceOptions opt_srv_get_entity_info =
            ros::AdvertiseServiceOptions::create<ed_gui_server::GetEntityInfo>(
                "/ed/gui/get_entity_info", boost::bind(&GUIServerPlugin::srvGetEntityInfo, this, _1, _2),
                ros::VoidPtr(), &cb_queue_);

    srv_get_entity_info_ = nh.advertiseService(opt_srv_get_entity_info);

    ros::AdvertiseServiceOptions opt_srv_interact =
            ros::AdvertiseServiceOptions::create<ed_gui_server::Interact>(
                "/ed/gui/interact", boost::bind(&GUIServerPlugin::srvInteract, this, _1, _2),
                ros::VoidPtr(), &cb_queue_);

    srv_interact_ = nh.advertiseService(opt_srv_interact);

    pub_entities_ = nh.advertise<ed_gui_server::EntityInfos>("/ed/gui/entities", 1);
}

// ----------------------------------------------------------------------------------------------------

void GUIServerPlugin::process(const ed::WorldModel& world, ed::UpdateRequest& req)
{
    world_model_ = &world;
    cb_queue_.callAvailable();

    ed_gui_server::EntityInfos entities_msg;
    for(ed::world_model::TransformCrawler tc(world, "map", world.latestTime()); tc.hasNext(); tc.next())
    {
        entities_msg.entities.push_back(ed_gui_server::EntityInfo());
        entityToMsg(tc.entity(), tc.transform(), entities_msg.entities.back());
    }

    robot_.getEntities(entities_msg.entities);

    pub_entities_.publish(entities_msg);
}

// ----------------------------------------------------------------------------------------------------

bool GUIServerPlugin::srvQueryEntities(const ed_gui_server::QueryEntities::Request& ros_req,
                                       ed_gui_server::QueryEntities::Response& ros_res)
{
    for(ed::world_model::TransformCrawler tc(*world_model_, "map", world_model_->latestTime()); tc.hasNext(); tc.next())
    {
        const ed::EntityConstPtr& e = tc.entity();
        const geo::Pose3D& e_pose = tc.transform();

        float pos_x = e_pose.t.x;
        float pos_y = e_pose.t.y;
        if (ros_req.area_min.x < pos_x && pos_x < ros_req.area_max.x
                && ros_req.area_min.y < pos_y && pos_y < ros_req.area_max.y)
        {
            ros_res.entities.push_back(ed_gui_server::EntityInfo());
            ed_gui_server::EntityInfo& info = ros_res.entities.back();

            info.id = e->id().str();
            info.mesh_revision = e->shapeRevision();
            geo::convert(e_pose, info.pose);
        }
    }

    return true;
}

// ----------------------------------------------------------------------------------------------------

enum ImageCompressionType
{
    IMAGE_COMPRESSION_JPG,
    IMAGE_COMPRESSION_PNG
};

bool imageToBinary(const cv::Mat& image, std::vector<unsigned char>& data, ImageCompressionType compression_type)
{
    if (compression_type == IMAGE_COMPRESSION_JPG)
    {
        // OpenCV compression settings
        std::vector<int> rgb_params;
        rgb_params.resize(3, 0);

        rgb_params[0] = CV_IMWRITE_JPEG_QUALITY;
        rgb_params[1] = 95; // default is 95

        // Compress image
        if (!cv::imencode(".jpg", image, data, rgb_params)) {
            std::cout << "RGB image compression failed" << std::endl;
            return false;
        }
    }
    else if (compression_type == IMAGE_COMPRESSION_PNG)
    {
        std::vector<int> params;
        params.resize(3, 0);

        params[0] = CV_IMWRITE_PNG_COMPRESSION;
        params[1] = 1;

        if (!cv::imencode(".png", image, data, params)) {
            std::cout << "PNG image compression failed" << std::endl;
            return false;
        }
    }

    return true;
}

// ----------------------------------------------------------------------------------------------------

bool GUIServerPlugin::srvGetEntityInfo(const ed_gui_server::GetEntityInfo::Request& ros_req,
                                       ed_gui_server::GetEntityInfo::Response& ros_res)
{
    ed::EntityConstPtr e = world_model_->getEntity(ros_req.id);
    if (!e)
        return true;

    ros_res.type = e->type();

    // TODO: get affordances from entity
    ros_res.affordances.push_back("navigate-to");
    ros_res.affordances.push_back("pick-up");
    ros_res.affordances.push_back("place");

//    std::stringstream ss_pose;
//    ss_pose << "(" << e->pose().t.x << ", " << e->pose().t.y << ", " << e->pose().t.z << ")";
//    ros_res.property_names.push_back("position");
//    ros_res.property_values.push_back(ss_pose.str());

    std::stringstream ss_creationTime;
    ss_creationTime << e->creationTime();
    ros_res.property_names.push_back("creation time");
    ros_res.property_values.push_back(ss_creationTime.str());

    ed::MeasurementConstPtr m = e->bestMeasurement();
    if (m)
    {
        const cv::Mat& rgb_image = m->image()->getRGBImage();
        const ed::ImageMask& image_mask = m->imageMask();

        cv::Mat rgb_image_masked(rgb_image.rows, rgb_image.cols, CV_8UC3, cv::Scalar(0, 0, 0));

        for(ed::ImageMask::const_iterator it = image_mask.begin(rgb_image.cols); it != image_mask.end(); ++it)
            rgb_image_masked.at<cv::Vec3b>(*it) = rgb_image.at<cv::Vec3b>(*it);

        imageToBinary(rgb_image_masked, ros_res.measurement_image, IMAGE_COMPRESSION_JPG);
    }

    return true;
}

// ----------------------------------------------------------------------------------------------------

bool GUIServerPlugin::srvQueryMeshes(const ed_gui_server::QueryMeshes::Request& ros_req,
                                      ed_gui_server::QueryMeshes::Response& ros_res)
{
    for(unsigned int i = 0; i < ros_req.entity_ids.size(); ++i)
    {
        const std::string& id = ros_req.entity_ids[i];

        geo::ShapeConstPtr shape = robot_.getShape(id);
        int shape_revision = 1;

        if (!shape)
        {
            ed::EntityConstPtr e = world_model_->getEntity(id);
            if (e)
            {
                shape = e->shape();
                shape_revision = e->shapeRevision();
            }
            else
                ros_res.error_msg += "Unknown entity: '" + id + "'.\n";
        }

        if (shape)
        {
            ros_res.entity_ids.push_back(id);

            ros_res.meshes.push_back(ed_gui_server::Mesh());
            ed_gui_server::Mesh& mesh_msg = ros_res.meshes.back();

            // Mesh revision
            mesh_msg.revision = shape_revision;

            const std::vector<geo::Vector3>& vertices = shape->getMesh().getPoints();

            // Triangles
            const std::vector<geo::TriangleI>& triangles = shape->getMesh().getTriangleIs();
            mesh_msg.vertices.resize(triangles.size() * 9);
            for(unsigned int i = 0; i < triangles.size(); ++i)
            {
                const geo::TriangleI& t = triangles[i];
                const geo::Vector3& v1 = vertices[t.i1_];
                const geo::Vector3& v2 = vertices[t.i2_];
                const geo::Vector3& v3 = vertices[t.i3_];

                unsigned int i9 = i * 9;

                mesh_msg.vertices[i9] = v1.x;
                mesh_msg.vertices[i9 + 1] = v1.y;
                mesh_msg.vertices[i9 + 2] = v1.z;
                mesh_msg.vertices[i9 + 3] = v2.x;
                mesh_msg.vertices[i9 + 4] = v2.y;
                mesh_msg.vertices[i9 + 5] = v2.z;
                mesh_msg.vertices[i9 + 6] = v3.x;
                mesh_msg.vertices[i9 + 7] = v3.y;
                mesh_msg.vertices[i9 + 8] = v3.z;
            }


            //                // Vertices
            //                const std::vector<geo::Vector3>& vertices = e->shape()->getMesh().getPoints();
            //                mesh_msg.vertices.resize(vertices.size() * 3);
            //                for(unsigned int i = 0; i < vertices.size(); ++i)
            //                {
            //                    mesh_msg.vertices[i * 3] = vertices[i].x;
            //                    mesh_msg.vertices[i * 3 + 1] = vertices[i].y;
            //                    mesh_msg.vertices[i * 3 + 2] = vertices[i].z;
            //                }

            //                // Triangles
            //                const std::vector<geo::TriangleI>& triangles = e->shape()->getMesh().getTriangleIs();
            //                mesh_msg.triangles.resize(triangles.size() * 3);
            //                for(unsigned int i = 0; i < triangles.size(); ++i)
            //                {
            //                    const geo::TriangleI& t = triangles[i];
            //                    mesh_msg.triangles[i * 3] = t.i1_;
            //                    mesh_msg.triangles[i * 3 + 1] = t.i2_;
            //                    mesh_msg.triangles[i * 3 + 2] = t.i3_;
            //                }
        }
    }

    return true;
}

// ----------------------------------------------------------------------------------------------------

void GUIServerPlugin::storeMeasurement(const std::string& id, const std::string& type)
{
    ed::EntityConstPtr e = world_model_->getEntity(id);
    if (e)
    {
        ed::MeasurementConstPtr msr = e->bestMeasurement();
        if (msr)
        {
            char const* home = getenv("HOME");
            if (home)
            {
                boost::filesystem::path dir(std::string(home) + "/.ed/measurements/" + type);
                boost::filesystem::create_directories(dir);
                ed::write(dir.string() + "/" + ed::Entity::generateID().str(), *msr);
            }
        }
        else
        {
            std::cout << "Entity '" + id << "' does not have any measurements." << std::endl;
        }
    }
    else
        std::cout << "Entity with id " << id << " does not exist!" << std::endl;
}

// ----------------------------------------------------------------------------------------------------

bool GUIServerPlugin::srvInteract(const ed_gui_server::Interact::Request& ros_req,
                                ed_gui_server::Interact::Response& ros_res)
{
    tue::Configuration params;
    tue::config::loadFromYAMLString(ros_req.command_yaml, params);

    std::string action;
    if (params.value("action", action))
    {
        if (action == "store")
        {
            std::string id, type;
            if (params.value("id", id) && params.value("type", type))
                storeMeasurement(id, type);
            else
                std::cout << "Please specify an id and a type!" << std::endl;
        }
    }

    if (params.hasError())
        ros_res.result_json = "{ error: \"" + params.error() + "\" }";
    else
        ros_res.result_json = "{}";
}

ED_REGISTER_PLUGIN(GUIServerPlugin)
