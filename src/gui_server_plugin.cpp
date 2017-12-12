#include "gui_server_plugin.h"

#include <ros/node_handle.h>
#include <ros/advertise_service_options.h>

#include <ed/world_model.h>
#include <ed/entity.h>
#include <ed/measurement.h>
#include <ed/io/filesystem/write.h>
#include <ed/error_context.h>

#include <rgbd/Image.h>

#include <geolib/Shape.h>
#include <geolib/ros/msg_conversions.h>

#include <tue/config/configuration.h>
#include <tue/config/loaders/yaml.h>
#include <tue/config/reader.h>

#include <ed_gui_server/EntityInfos.h>
#include <ed/serialization/serialization.h>


#include <boost/filesystem.hpp>

#include <opencv2/highgui/highgui.hpp>

void GUIServerPlugin::entityToMsg(const ed::EntityConstPtr& e, ed_gui_server::EntityInfo& msg)
{
    ed::ErrorContext errc("entityToMsg");

    msg.id = e->id().str();
    msg.type = e->type();
    msg.existence_probability = e->existenceProbability();
    msg.mesh_revision = e->shapeRevision();

    if (e->has_pose())
    {
        geo::convert(e->pose(), msg.pose);
        msg.has_pose = true;
    }
    else
    {
        msg.has_pose = false;
    }

    if (!e->shape())
    {
        const ed::ConvexHull& ch = e->convexHull();

        if (!ch.points.empty() && e->has_pose())
        {
            const geo::Pose3D& pose = e->pose();

            geo::convert(pose, msg.pose);
            msg.has_pose = true;

            msg.polygon.z_min = ch.z_min;
            msg.polygon.z_max = ch.z_max;

            unsigned int size = ch.points.size();
            msg.polygon.xs.resize(size);
            msg.polygon.ys.resize(size);

            for(unsigned int i = 0; i < size; ++i)
            {
                msg.polygon.xs[i] = ch.points[i].x;
                msg.polygon.ys[i] = ch.points[i].y;
            }
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

    if (e->hasFlag("highlighted"))
    {
        msg.color.a = 255;
        msg.color.r = 255;
        msg.color.g = 0;
        msg.color.b = 0;
    }

    // HACK! Way of coding that this is a human
    if (e->type() == "human" || e->hasFlag("possible_human"))
    {
        msg.color.a = 1;
        msg.color.r = 2;
        msg.color.g = 3;

        if (e->type() == "human")
            msg.color.b = 4;  // human
        else
            msg.color.b = 5;  // possible human (based on laser)
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

void GUIServerPlugin::initialize(ed::InitData& init)
{
    ed::ErrorContext errc("initialize");

    tue::Configuration& config = init.config;

    std::string robot_name;
    if (config.value("robot_name", robot_name, tue::OPTIONAL))
    {
        robot_.initialize(robot_name);
    }

    ros::NodeHandle nh;

    ros::AdvertiseServiceOptions opt_srv_entities =
            ros::AdvertiseServiceOptions::create<ed_gui_server::QueryEntities>(
                "ed/gui/query_entities", boost::bind(&GUIServerPlugin::srvQueryEntities, this, _1, _2),
                ros::VoidPtr(), &cb_queue_);

    srv_query_entities_ = nh.advertiseService(opt_srv_entities);


    ros::AdvertiseServiceOptions opt_srv_meshes =
            ros::AdvertiseServiceOptions::create<ed_gui_server::QueryMeshes>(
                "ed/gui/query_meshes", boost::bind(&GUIServerPlugin::srvQueryMeshes, this, _1, _2),
                ros::VoidPtr(), &cb_queue_);

    srv_query_meshes_ = nh.advertiseService(opt_srv_meshes);

    ros::AdvertiseServiceOptions opt_srv_get_entity_info =
            ros::AdvertiseServiceOptions::create<ed_gui_server::GetEntityInfo>(
                "ed/gui/get_entity_info", boost::bind(&GUIServerPlugin::srvGetEntityInfo, this, _1, _2),
                ros::VoidPtr(), &cb_queue_);

    srv_get_entity_info_ = nh.advertiseService(opt_srv_get_entity_info);

    ros::AdvertiseServiceOptions opt_srv_interact =
            ros::AdvertiseServiceOptions::create<ed_gui_server::Interact>(
                "ed/gui/interact", boost::bind(&GUIServerPlugin::srvInteract, this, _1, _2),
                ros::VoidPtr(), &cb_queue_);

    srv_interact_ = nh.advertiseService(opt_srv_interact);

    pub_entities_ = nh.advertise<ed_gui_server::EntityInfos>("ed/gui/entities", 1);
}

// ----------------------------------------------------------------------------------------------------

void GUIServerPlugin::process(const ed::WorldModel& world, ed::UpdateRequest& req)
{
    ed::ErrorContext errc("process");

    world_model_ = &world;
    cb_queue_.callAvailable();

    ed_gui_server::EntityInfos entities_msg;

    entities_msg.header.stamp = ros::Time::now();
    entities_msg.header.frame_id = "/map";

    entities_msg.entities.resize(world_model_->numEntities());

    unsigned int i = 0;
    for(ed::WorldModel::const_iterator it = world_model_->begin(); it != world_model_->end(); ++it)
    {
        const ed::EntityConstPtr& e = *it;
        if (!e->hasFlag("self"))
            entityToMsg(e, entities_msg.entities[i++]);
    }

    robot_.getEntities(entities_msg.entities);

    pub_entities_.publish(entities_msg);
}

// ----------------------------------------------------------------------------------------------------

bool GUIServerPlugin::srvQueryEntities(const ed_gui_server::QueryEntities::Request& ros_req,
                                       ed_gui_server::QueryEntities::Response& ros_res)
{
    for(ed::WorldModel::const_iterator it = world_model_->begin(); it != world_model_->end(); ++it)
    {
        const ed::EntityConstPtr& e = *it;

        const geo::Pose3D* pose = 0;
        if (e->has_pose())
            pose = &e->pose();

        if (!pose)
            continue;

        float pos_x = pose->t.x;
        float pos_y = pose->t.y;

        if (ros_req.area_min.x < pos_x && pos_x < ros_req.area_max.x
                && ros_req.area_min.y < pos_y && pos_y < ros_req.area_max.y)
        {
            ros_res.entities.push_back(ed_gui_server::EntityInfo());
            ed_gui_server::EntityInfo& info = ros_res.entities.back();

            info.id = e->id().str();
            info.mesh_revision = e->shapeRevision();
            geo::convert(*pose, info.pose);
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

    const geo::Pose3D* pose = 0;
    if (e->has_pose())
        pose = &e->pose();

    if (pose)
    {
        std::stringstream ss_pose;
        ss_pose << "(" << pose->t.x << ", " << pose->t.y << ", " << pose->t.z << ")";
        ros_res.property_names.push_back("position");
        ros_res.property_values.push_back(ss_pose.str());
    }

//    std::stringstream ss_creationTime;
//    ss_creationTime << e->creationTime();
//    ros_res.property_names.push_back("creation time");
//    ros_res.property_values.push_back(ss_creationTime.str());

    ed::MeasurementConstPtr m = e->lastMeasurement();
    if (m)
    {
        const cv::Mat& rgb_image = m->image()->getRGBImage();
        const ed::ImageMask& image_mask = m->imageMask();

        cv::Mat rgb_image_masked(rgb_image.rows, rgb_image.cols, CV_8UC3, cv::Scalar(0, 0, 0));

        cv::Point min(rgb_image.cols, rgb_image.rows);
        cv::Point max(0, 0);
        for(ed::ImageMask::const_iterator it = image_mask.begin(rgb_image.cols); it != image_mask.end(); ++it)
        {
            const cv::Point2i p(it());
            rgb_image_masked.at<cv::Vec3b>(p) = rgb_image.at<cv::Vec3b>(p);

            min.x = std::min(min.x, p.x);
            min.y = std::min(min.y, p.y);
            max.x = std::max(max.x, p.x);
            max.y = std::max(max.y, p.y);
        }

        imageToBinary(rgb_image_masked, ros_res.measurement_image, IMAGE_COMPRESSION_JPG);

        // Add border to roi
        min.x = std::max(0, min.x - ros_req.measurement_image_border);
        min.y = std::max(0, min.y - ros_req.measurement_image_border);
        max.x = std::min(rgb_image.cols - 1, max.x + ros_req.measurement_image_border);
        max.y = std::min(rgb_image.rows - 1, max.y + ros_req.measurement_image_border);

        imageToBinary(rgb_image(cv::Rect(min, max)), ros_res.measurement_image_unmasked, IMAGE_COMPRESSION_JPG);
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

        ed::EntityConstPtr e;
        if (!shape)
        {
            e = world_model_->getEntity(id);
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
            ros_res.entity_geometries.push_back(ed_gui_server::EntityMeshAndAreas());
            ed_gui_server::EntityMeshAndAreas& entity_geometry = ros_res.entity_geometries.back();

            entity_geometry.id = id;

            // Mesh revision
            entity_geometry.mesh.revision = shape_revision;

            const std::vector<geo::Vector3>& vertices = shape->getMesh().getPoints();

            // Triangles
            const std::vector<geo::TriangleI>& triangles = shape->getMesh().getTriangleIs();
            entity_geometry.mesh.vertices.resize(triangles.size() * 9);
            for(unsigned int i = 0; i < triangles.size(); ++i)
            {
                const geo::TriangleI& t = triangles[i];
                const geo::Vector3& v1 = vertices[t.i1_];
                const geo::Vector3& v2 = vertices[t.i2_];
                const geo::Vector3& v3 = vertices[t.i3_];

                unsigned int i9 = i * 9;

                entity_geometry.mesh.vertices[i9] = v1.x;
                entity_geometry.mesh.vertices[i9 + 1] = v1.y;
                entity_geometry.mesh.vertices[i9 + 2] = v1.z;
                entity_geometry.mesh.vertices[i9 + 3] = v2.x;
                entity_geometry.mesh.vertices[i9 + 4] = v2.y;
                entity_geometry.mesh.vertices[i9 + 5] = v2.z;
                entity_geometry.mesh.vertices[i9 + 6] = v3.x;
                entity_geometry.mesh.vertices[i9 + 7] = v3.y;
                entity_geometry.mesh.vertices[i9 + 8] = v3.z;
            }

            // Render areas if e
            if (e)
            {
                geo::Shape area_shape;
                tue::config::Reader r(e->data());

                if (r.readArray("areas"))
                {
                    while(r.nextArrayItem())
                    {
                        std::string a_name;
                        if (!r.value("name", a_name))
                            continue;

                        if (ed::deserialize(r, "shape", area_shape))
                        {
                            entity_geometry.areas.push_back(ed_gui_server::Area());
                            ed_gui_server::Area& entity_area = entity_geometry.areas.back();

                            entity_area.name = a_name;

                            const std::vector<geo::Vector3>& vertices = area_shape.getMesh().getPoints();

                            // Triangles

                            const std::vector<geo::TriangleI>& triangles = area_shape.getMesh().getTriangleIs();
                            entity_area.mesh.vertices.resize(triangles.size() * 9);
                            for(unsigned int i = 0; i < triangles.size(); ++i)
                            {
                                const geo::TriangleI& t = triangles[i];
                                const geo::Vector3& v1 = vertices[t.i1_];
                                const geo::Vector3& v2 = vertices[t.i2_];
                                const geo::Vector3& v3 = vertices[t.i3_];

                                unsigned int i9 = i * 9;

                                entity_area.mesh.vertices[i9] = v1.x;
                                entity_area.mesh.vertices[i9 + 1] = v1.y;
                                entity_area.mesh.vertices[i9 + 2] = v1.z;
                                entity_area.mesh.vertices[i9 + 3] = v2.x;
                                entity_area.mesh.vertices[i9 + 4] = v2.y;
                                entity_area.mesh.vertices[i9 + 5] = v2.z;
                                entity_area.mesh.vertices[i9 + 6] = v3.x;
                                entity_area.mesh.vertices[i9 + 7] = v3.y;
                                entity_area.mesh.vertices[i9 + 8] = v3.z;
                            }
                        }
                    }
                    r.endArray();
                }
            }
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
        char const* home = getenv("HOME");
        if (home)
        {
            boost::filesystem::path dir(std::string(home) + "/.ed/measurements/" + type);
            boost::filesystem::create_directories(dir);

            std::string filename = dir.string() + "/" + ed::Entity::generateID().str();
            ed::write(filename, *e);

            std::cout << "Writing entity info to '" << filename << "'." << std::endl;
        }
    }
    else
        std::cout << "Entity with id " << id << " does not exist!" << std::endl;
}

// ----------------------------------------------------------------------------------------------------

bool GUIServerPlugin::srvInteract(const ed_gui_server::Interact::Request& ros_req,
                                ed_gui_server::Interact::Response& ros_res)
{
    std::cout << "[GUIServerPlugin] Received command: " << ros_req.command_yaml << std::endl;

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
