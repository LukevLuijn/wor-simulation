//
// Created by luke on 24-10-22.
//

#ifndef WOR_SIMULATION_ARM_NODE_H
#define WOR_SIMULATION_ARM_NODE_H

#include "rclcpp/rclcpp.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include "simulation_msgs/msg/command.hpp"
#include "simulation_msgs/msg/cup_pickup.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/point.hpp"

class ArmNode : public rclcpp::Node {

    typedef simulation_msgs::msg::Command Command;
    typedef simulation_msgs::msg::CupPickup CupPickup;
    typedef sensor_msgs::msg::JointState JointState;
    typedef geometry_msgs::msg::TransformStamped Transform;


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

    bool parseCommandString(const std::string &command, std::vector<ServoCommand> &buffer) const;

    void updateJointState();

    void updateTransform();

    bool canPickupCup();

private:
    void initJointState();

private:
    std::string sim_link_, bot_link_, cup_link_;
    bool is_holding_cup_;

    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener listener_;
    tf2_ros::TransformBroadcaster broadcaster_;

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<CupPickup>::SharedPtr cup_pickup_pub_;
    rclcpp::Subscription<Command>::SharedPtr command_sub_;

    JointState joint_state_message_;
    Transform transform_stamped_;
};

#endif //WOR_SIMULATION_ARM_NODE_H
