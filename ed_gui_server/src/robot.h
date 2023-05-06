#ifndef ED_GUI_SERVER_ROBOT_H_
#define ED_GUI_SERVER_ROBOT_H_

#include <ed_gui_server_msgs/Color.h>
#include <ed_gui_server_msgs/EntityInfo.h>

#include <ed/types.h>

#include <geolib/datatypes.h>

#include <memory>

struct Visual
{
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

    Robot(const ed::TFBufferConstPtr& tf_buffer);

    virtual ~Robot();

    void initialize(const std::string& name, const std::string& urdf_rosparam, const std::string& tf_prefix);

    inline const std::string& name() const { return name_; }

    geo::ShapeConstPtr getShape(const std::string& id) const;

    bool contains(const std::string& id) const;

    void getEntities(std::vector<ed_gui_server_msgs::EntityInfo>& entities) const;

private:

    std::string name_;

    std::string urdf_rosparam_;

    std::string tf_prefix_;

    ed::TFBufferConstPtr tf_buffer_;

    ShapeMap shapes_;

};

} // end namespace gui

#endif
