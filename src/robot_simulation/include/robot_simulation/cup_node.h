//
// Created by luke on 21-10-22.
//

#ifndef WOR_SIMULATION_CUP_NODE_H
#define WOR_SIMULATION_CUP_NODE_H

#include "rclcpp/rclcpp.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include "geometry_msgs/msg/transform_stamped.hpp"

#include "visualization_msgs/msg/marker.hpp"
#include "simulation_msgs/msg/speed.hpp"
#include "simulation_msgs/msg/pose.hpp"
#include "simulation_msgs/msg/state.hpp"


class CupNode : public rclcpp::Node {

    typedef simulation_msgs::msg::State State;
    typedef simulation_msgs::msg::Pose Pose;
    typedef simulation_msgs::msg::Speed Speed;
    typedef visualization_msgs::msg::Marker Marker;

public:
    CupNode();

    virtual ~CupNode() = default;

private:
    void timerCallback();

    void gripperCallback(const State::SharedPtr message);

    void updateMarker();

    void updateTransform();

    void updateTopics();

private:
    void initMarker();

    void initTransform();

private:
    const double GRAVITY = 9.81;

    std::string sim_link_, bot_link_, cup_link_;
    double velocity_;

    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener listener_;
    tf2_ros::TransformBroadcaster broadcaster_;
    geometry_msgs::msg::TransformStamped transform_;

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<Pose>::SharedPtr pose_pub_;
    rclcpp::Publisher<Speed>::SharedPtr speed_pub_;

    Marker marker_message_;

    rclcpp::Subscription<State>::SharedPtr state_sub_;

};

#endif //WOR_SIMULATION_CUP_NODE_H
