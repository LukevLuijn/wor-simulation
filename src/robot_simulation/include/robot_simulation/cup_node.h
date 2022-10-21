//
// Created by luke on 21-10-22.
//

#ifndef WOR_SIMULATION_CUP_NODE_H
#define WOR_SIMULATION_CUP_NODE_H

#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"


class CupNode : public rclcpp::Node {

public:
    CupNode();

    virtual ~CupNode() = default;

private:
    void timerCallback();

    void initNode();

    void updateMarker();

    void initMarker();

    void updateTransform();

    void initTransform();

private:
    const double GRAVITY = 9.81;

    std::string sim_link_, bot_link_, cup_link_;
    double velocity_;

    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener listener_;
    tf2_ros::TransformBroadcaster broadcaster_;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    visualization_msgs::msg::Marker marker_;
    geometry_msgs::msg::TransformStamped transform_;
};

#endif //WOR_SIMULATION_CUP_NODE_H
