//
// Created by luke on 24-10-22.
//

#ifndef WOR_SIMULATION_ARM_NODE_H
#define WOR_SIMULATION_ARM_NODE_H

#include "rclcpp/rclcpp.hpp"

class ArmNode : public rclcpp::Node {
public:
    ArmNode();

    virtual ~ArmNode() = default;

private:
    void timerCallback();

private:
    std::string sim_link_, bot_link_, cup_link_;
    rclcpp::TimerBase::SharedPtr timer_;

};

#endif //WOR_SIMULATION_ARM_NODE_H
