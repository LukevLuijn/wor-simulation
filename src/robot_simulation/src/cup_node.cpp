#include "cup_node.h"

#include <ament_index_cpp/get_package_share_directory.hpp>

CupNode::CupNode()
        : Node("cup_node"), velocity_(0.0), buffer_(get_clock()), listener_(buffer_), broadcaster_(this) {

    RCLCPP_INFO(this->get_logger(), "Hello world"); // TODO for testing

    this->declare_parameter<double>("pos_x", 0.0);
    this->declare_parameter<double>("pos_y", 0.0);
    this->declare_parameter<double>("pos_z", 0.0);

    this->declare_parameter<std::string>("sim_link_name", "sim_link"); // world
    this->declare_parameter<std::string>("bot_link_name", "bot_link"); // robot
    this->declare_parameter<std::string>("cup_link_name", "cup_link"); // cup

    publisher_ = create_publisher<visualization_msgs::msg::Marker>("cup_marker", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(10), [this] { timerCallback(); });

    initNode();
    initTransform();
    initMarker();
}

void CupNode::timerCallback() {
    updateTransform();
    updateMarker();
}

void CupNode::initNode() {
    sim_link_ = "sim_link";//this->get_parameter("sim_link_name").get_parameter_value().get<std::string>();
    bot_link_ = "bot_link";//this->get_parameter("bot_link_name").get_parameter_value().get<std::string>();
    cup_link_ = "cup_link";//this->get_parameter("cup_link_name").get_parameter_value().get<std::string>();
}

void CupNode::updateMarker() {
    marker_.header.stamp = now();

    marker_.color.r = 1.0f;
    marker_.color.g = 1.0f;
    marker_.color.b = 1.0f;
    marker_.color.a = 1.0f;

    marker_.pose.position.x = transform_.transform.translation.x;//this->get_parameter("pos_x").get_parameter_value().get<double>();
    marker_.pose.position.y = transform_.transform.translation.y;//this->get_parameter("pos_y").get_parameter_value().get<double>();
    marker_.pose.position.z = transform_.transform.translation.z;//this->get_parameter("pos_z").get_parameter_value().get<double>();

//    marker_.pose.position.x = this->get_parameter("pos_x").get_parameter_value().get<double>();
//    marker_.pose.position.y = this->get_parameter("pos_y").get_parameter_value().get<double>();
//    marker_.pose.position.z = transform_.transform.translation.z;

    marker_.pose.orientation.x = 0.0;
    marker_.pose.orientation.y = 0.0;
    marker_.pose.orientation.z = 0.0;
    marker_.pose.orientation.w = 1.0;

    publisher_->publish(marker_);
}

void CupNode::initMarker() {
    marker_ = visualization_msgs::msg::Marker();

    marker_.header.frame_id = cup_link_; //this->get_parameter("cup_link_name").get_parameter_value().get<std::string>();
    marker_.header.stamp = now();

    marker_.ns = "water_cup";
    marker_.id = 0;

    marker_.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    marker_.action = visualization_msgs::msg::Marker::ADD;

    std::string package_share_directory = ament_index_cpp::get_package_share_directory("robot_simulation");
    std::string file_name = package_share_directory.append("/model/wor_sim_cup.stl");

    marker_.mesh_resource = "file://" + file_name;

    // .stl scaling from meters to millimeters.
    marker_.scale.x = 0.001;
    marker_.scale.y = 0.001;
    marker_.scale.z = 0.001;

    marker_.color.r = 1.0f;
    marker_.color.g = 1.0f;
    marker_.color.b = 1.0f;
    marker_.color.a = 1.0f;

    marker_.pose.position.x = this->get_parameter("pos_x").get_parameter_value().get<double>();
    marker_.pose.position.y = this->get_parameter("pos_y").get_parameter_value().get<double>();
    marker_.pose.position.z = this->get_parameter("pos_z").get_parameter_value().get<double>();

    marker_.pose.orientation.x = 0.0;
    marker_.pose.orientation.y = 0.0;
    marker_.pose.orientation.z = 0.0;
    marker_.pose.orientation.w = 1.0;

    marker_.lifetime = builtin_interfaces::msg::Duration();
}

void CupNode::updateTransform() {

    auto timeInSeconds = [](const rclcpp::Duration &time) -> double {
        return static_cast<double>(time.nanoseconds()) / 1'000'000'000.0;
    };

    geometry_msgs::msg::TransformStamped previous_transform;
    previous_transform = buffer_.lookupTransform(sim_link_, cup_link_, rclcpp::Time(0));

    if (previous_transform.transform.translation.z > 0) // cup is in the air
    {
        rclcpp::Duration duration = now() - previous_transform.header.stamp;
        double previous_velocity = velocity_;
        velocity_ += GRAVITY * timeInSeconds(duration);
        double average_velocity = ((previous_velocity + velocity_) / 2) * timeInSeconds(duration);
        double new_trans_z = std::max(previous_transform.transform.translation.z - average_velocity, 0.0);

        if (new_trans_z == 0)
        {
            velocity_ = 0;
        }
        transform_.transform.translation.z = new_trans_z;
    } else if (previous_transform.transform.translation.z < 0) // cup is below the ground
    {
        RCLCPP_INFO(this->get_logger(), "does this happen?");
        // bring back to 0?
    }
//    else
//    {
////        RCLCPP_INFO(this->get_logger(), "cup is not falling");
//
//    }

    transform_.header.stamp = now();
    broadcaster_.sendTransform(transform_);
}

void CupNode::initTransform() {
    transform_.header.frame_id = sim_link_;//"world";//this->get_parameter("sim_link_name").get_parameter_value().get<std::string>();
    transform_.child_frame_id = cup_link_;//this->get_parameter("cup_link_name").get_parameter_value().get<std::string>();
    transform_.header.stamp = now();

    transform_.transform.translation.x = 0.25;
    transform_.transform.translation.y = 0.25;
    transform_.transform.translation.z = 10;

    broadcaster_.sendTransform(transform_);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CupNode>());
    rclcpp::shutdown();
    return 0;
}
