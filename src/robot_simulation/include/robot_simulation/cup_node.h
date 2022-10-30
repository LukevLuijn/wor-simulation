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
#include "std_msgs/msg/color_rgba.hpp"

#include "simulation_msgs/msg/speed.hpp"
#include "simulation_msgs/msg/position.hpp"
#include "simulation_msgs/msg/cup_pickup.hpp"

class CupNode : public rclcpp::Node {

    typedef simulation_msgs::msg::CupPickup CupPickup;
    typedef simulation_msgs::msg::Position Position;
    typedef simulation_msgs::msg::Speed Speed;

    typedef geometry_msgs::msg::TransformStamped Transform;

    typedef visualization_msgs::msg::Marker Marker;
    typedef std_msgs::msg::ColorRGBA Color;

public:
    CupNode();

    virtual ~CupNode() = default;

private:
    void baseTimerCallback();

    void speedTimerCallback();

    void cupPickupCallback(const CupPickup::SharedPtr message);

    void updateMarker();

    void updateTransform();

    void updatePosition();

    void updateTopics();

    void initMarker();

    void initTransform();

    static double timeInSeconds(const rclcpp::Duration &time) {
        return static_cast<double>(time.nanoseconds()) / 1'000'000'000.0;
    }

private:
    const double GRAVITY = 9.81;
    Color default_color_;
    Color picked_up_color_;

    std::string sim_link_, bot_link_, cup_link_;
    double velocity_;
    bool cup_picked_up_;

    Transform transform_;
    Transform previous_tf_;
    Marker marker_message_;
    Speed speed_message_;

    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener listener_;
    tf2_ros::TransformBroadcaster broadcaster_;

    rclcpp::TimerBase::SharedPtr base_timer_;
    rclcpp::TimerBase::SharedPtr speed_timer_;

    rclcpp::Publisher<Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<Position>::SharedPtr position_pub_;
    rclcpp::Publisher<Speed>::SharedPtr speed_pub_;

    rclcpp::Subscription<CupPickup>::SharedPtr cup_pickup_sub_;
};

#endif //WOR_SIMULATION_CUP_NODE_H
