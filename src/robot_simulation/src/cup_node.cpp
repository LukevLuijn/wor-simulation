#include "cup_node.h"

#include "geometry_msgs/msg/point.hpp"

#include <functional>

using namespace std::chrono_literals;

CupNode::CupNode() : Node("cup_node") {

    RCLCPP_INFO(this->get_logger(), "Hello world");

    this->declare_parameter<double>("posX", 0.0);
    this->declare_parameter<double>("posY", 0.0);
    this->declare_parameter<double>("posZ", 0.0);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), [this] { timerCallback(); });
}

void CupNode::timerCallback() {
    geometry_msgs::msg::Point point;

    point.x = this->get_parameter("posX").get_parameter_value().get<double>();
    point.y = this->get_parameter("posY").get_parameter_value().get<double>();
    point.z = this->get_parameter("posZ").get_parameter_value().get<double>();

    RCLCPP_INFO(this->get_logger(), "position: ['%f', '%f', '%f']", point.x, point.y, point.z);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CupNode>());
    rclcpp::shutdown();
    return 0;
}
