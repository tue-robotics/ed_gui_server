#ifndef ED_GUI_SERVER_ROBOT_H_
#define ED_GUI_SERVER_ROBOT_H_

#include <ed_gui_server_msgs/Color.h>
#include <ed_gui_server_msgs/EntityInfo.h>

#include <geolib/datatypes.h>

#include <tf2_ros/buffer.h>

#include <memory>

namespace tf2_ros {
    class TransformListener;
}

struct Visual
{
    geo::Pose3D offset;
    geo::ShapeConstPtr shape;
    ed_gui_server_msgs::Color color;
    std::string link;
};

namespace gui
{

typedef std::map<std::string, Visual> ShapeMap;

class Robot
{

public:

    Robot();

    virtual ~Robot();

    void initialize(const std::string& name, const std::string& urdf_rosparam, const std::string& tf_prefix);

    inline const std::string& name() const { return name_; }

    geo::ShapeConstPtr getShape(const std::string& id) const;

    void getEntities(std::vector<ed_gui_server_msgs::EntityInfo>& entities) const;

private:

    std::string name_;

    std::string urdf_rosparam_;

    std::string tf_prefix_;

    tf2_ros::Buffer tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

    ShapeMap shapes_;

private:

};

} // end namespace gui

#endif
