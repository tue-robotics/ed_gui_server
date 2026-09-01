#ifndef ED_GUI_SERVER_ROBOT_H_
#define ED_GUI_SERVER_ROBOT_H_

#include <ed/types.h>

#include <ed_gui_server_interfaces/msg/color.hpp>
#include <ed_gui_server_interfaces/msg/entity_info.hpp>

#include <geolib/datatypes.h>

#include <rclcpp/node.hpp>

#include <map>
#include <string>
#include <vector>

struct Visual
{
    geo::ShapeConstPtr shape;
    ed_gui_server_interfaces::msg::Color color;
    std::string link;
};

namespace gui
{

using ShapeMap = std::map<std::string, Visual>;

class Robot
{

public:
    Robot(rclcpp::Node::SharedPtr node, ed::TFBufferConstPtr tf_buffer);

    virtual ~Robot();

    void initialize(const std::string& name, const std::string& urdf_rosparam, const std::string& tf_prefix);

    [[nodiscard]]
    const std::string& name() const
    {
        return name_;
    }

    [[nodiscard]]
    geo::ShapeConstPtr getShape(const std::string& id) const;

    [[nodiscard]]
    bool contains(const std::string& id) const;

    void getEntities(std::vector<ed_gui_server_interfaces::msg::EntityInfo>& entities) const;

private:
    std::string name_;

    std::string urdf_rosparam_;

    std::string tf_prefix_;

    rclcpp::Node::SharedPtr node_;

    ed::TFBufferConstPtr tf_buffer_;

    ShapeMap shapes_;
};

} // namespace gui

#endif
