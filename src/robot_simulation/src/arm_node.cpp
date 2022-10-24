//
// Created by luke on 24-10-22.
//

#include "arm_node.h"

ArmNode::ArmNode() : Node ("arm_node")
{
    RCLCPP_INFO(this->get_logger(), "Hello world from arm node!"); // TODO for testing

    this->declare_parameter<double>("pos_x", 0.0);
    this->declare_parameter<double>("pos_y", 0.0);
    this->declare_parameter<double>("pos_z", 0.0);

    this->declare_parameter<std::string>("sim_link_name", "sim_link"); // world
    this->declare_parameter<std::string>("bot_link_name", "bot_link"); // robot
    this->declare_parameter<std::string>("cup_link_name", "cup_link"); // cup

    timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&ArmNode::timerCallback, this));

    sim_link_ = this->get_parameter("sim_link_name").get_parameter_value().get<std::string>();//"sim_link";
    bot_link_ = this->get_parameter("bot_link_name").get_parameter_value().get<std::string>();//"bot_link";
    cup_link_ = this->get_parameter("cup_link_name").get_parameter_value().get<std::string>();//"cup_link";
}
void ArmNode::timerCallback()
{
    RCLCPP_INFO(this->get_logger(), "arm node is online..");
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmNode>());
    rclcpp::shutdown();
    return 0;
}