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

    /**
     * @brief Wait for the robot description and build the shapes of all links with a visual
     * @param name Name of the robot, used to prefix the entity ids
     * @param urdf_topic Topic publishing the URDF, latched (transient local), e.g. by
     * robot_state_publisher
     * @param tf_prefix Prefix of the link names in tf
     * @param error Reason for failure, only set when this returns false
     * @return whether the model could be built
     */
    bool initialize(const std::string& name,
                    const std::string& urdf_topic,
                    const std::string& tf_prefix,
                    std::string& error);

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
    /**
     * @brief Build the shapes of all links with a visual
     * @param urdf_xml Robot description
     * @param error Reason for failure, only set when this returns false
     * @return whether the model could be built
     */
    bool buildModel(const std::string& urdf_xml, std::string& error);

    std::string name_;

    std::string tf_prefix_;

    rclcpp::Node::SharedPtr node_;

    ed::TFBufferConstPtr tf_buffer_;

    ShapeMap shapes_;
};

} // namespace gui

#endif
