//
// Created by luke on 21-10-22.
//

#ifndef WOR_SIMULATION_CUP_NODE_H
#define WOR_SIMULATION_CUP_NODE_H

#include "rclcpp/rclcpp.hpp"

class CupNode : public rclcpp::Node {

public:
    CupNode();

    virtual ~CupNode() = default;

private:
    void timerCallback();

private:
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif //WOR_SIMULATION_CUP_NODE_H
