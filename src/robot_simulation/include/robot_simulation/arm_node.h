//
// Created by luke on 24-10-22.
//

#ifndef WOR_SIMULATION_ARM_NODE_H
#define WOR_SIMULATION_ARM_NODE_H

#include "rclcpp/rclcpp.hpp"

#include "tf2_ros/transform_broadcaster.h"

#include "simulation_msgs/msg/command.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class ArmNode : public rclcpp::Node {

    typedef simulation_msgs::msg::Command Command;
    typedef sensor_msgs::msg::JointState JointState;

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

    void updateJointState();
    void updateTransform();

private:
    void initJointState();

private:
    std::string sim_link_, bot_link_, cup_link_;

    tf2_ros::TransformBroadcaster broadcaster_;

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<JointState>::SharedPtr joint_state_pub_;
    rclcpp::Subscription<Command>::SharedPtr command_sub_;

    JointState joint_state_message_;

};

#endif //WOR_SIMULATION_ARM_NODE_H
