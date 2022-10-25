//
// Created by luke on 24-10-22.
//

#ifndef WOR_SIMULATION_ARM_NODE_H
#define WOR_SIMULATION_ARM_NODE_H

#include "rclcpp/rclcpp.hpp"
#include "simulation_msgs/msg/command.hpp"

class ArmNode : public rclcpp::Node {

    typedef simulation_msgs::msg::Command Command;
    struct ServoCommand {
        uint8_t index_;
        uint16_t position_;
        uint16_t duration_;
    };

public:
    ArmNode();

    virtual ~ArmNode() = default;

private:
    void timerCallback();

    void commandCallback(const Command::SharedPtr command_msg);
    bool parseCommandString(const std::string& command, std::vector<ServoCommand>& buffer) const;

private:
    std::string sim_link_, bot_link_, cup_link_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<Command>::SharedPtr command_sub_;

};

#endif //WOR_SIMULATION_ARM_NODE_H
