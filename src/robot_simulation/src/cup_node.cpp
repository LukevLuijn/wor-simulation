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

    timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&CupNode::timerCallback, this));
    state_sub_ = this->create_subscription<State>("sim/arm/gripper", 10,
                                            std::bind(&CupNode::gripperCallback, this, std::placeholders::_1));

    marker_pub_ = create_publisher<Marker>("sim/cup/marker", 10);
    pose_pub_ = create_publisher<Pose>("sim/cup/pose", 10);
    speed_pub_ = create_publisher<Speed>("sim/cup/speed", 10);

    sim_link_ = this->get_parameter("sim_link_name").get_parameter_value().get<std::string>();//"sim_link";
    bot_link_ = this->get_parameter("bot_link_name").get_parameter_value().get<std::string>();//"bot_link";
    cup_link_ = this->get_parameter("cup_link_name").get_parameter_value().get<std::string>();//"cup_link";

    initTransform();
    initMarker();
}

void CupNode::timerCallback() {

    updateTransform();
    updateMarker();
    updateTopics();
}

void CupNode::gripperCallback(const State::SharedPtr message) {

    if (message->state == State::CLOSED) {

        transform_.transform.translation.z = 10; // to test gravity

        RCLCPP_INFO(this->get_logger(), "gripper is closed.");
    } else {
        // do nothing
        RCLCPP_INFO(this->get_logger(), "gripper is open.");
    }

}

void CupNode::updateMarker() {

    marker_message_.pose.position.x = transform_.transform.translation.x;
    marker_message_.pose.position.y = transform_.transform.translation.y;
    marker_message_.pose.position.z = transform_.transform.translation.z;

    // based on movement
    marker_message_.pose.orientation.x = 0.0;
    marker_message_.pose.orientation.y = 0.0;
    marker_message_.pose.orientation.z = 0.0;
    marker_message_.pose.orientation.w = 1.0;

    // based on tilt of cup
    marker_message_.color.r = 1.0f;
    marker_message_.color.g = 1.0f;
    marker_message_.color.b = 1.0f;
    marker_message_.color.a = 1.0f;

    marker_message_.header.stamp = now();
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

        if (new_trans_z == 0) {
            velocity_ = 0;
        }
        transform_.transform.translation.z = new_trans_z;
    } else if (previous_transform.transform.translation.z < 0) // cup is below the ground
    {
        RCLCPP_INFO(this->get_logger(), "does this happen?");
        // bring back to 0?
    }

    transform_.header.stamp = now();
}

void CupNode::updateTopics()
{
    simulation_msgs::msg::Speed speed;
    speed.velocity = velocity_;

    speed_pub_->publish(speed);

    simulation_msgs::msg::Pose pose;
    pose.pose = marker_message_.pose;

    pose_pub_->publish(pose);

    marker_pub_->publish(marker_message_);
    broadcaster_.sendTransform(transform_);
}

void CupNode::initTransform() {
    transform_.header.frame_id = sim_link_;
    transform_.child_frame_id = cup_link_;
    transform_.header.stamp = now();

    transform_.transform.translation.x = this->get_parameter("pos_x").get_parameter_value().get<double>();//0.25;
    transform_.transform.translation.y = this->get_parameter("pos_y").get_parameter_value().get<double>();//0.25;
    transform_.transform.translation.z = this->get_parameter("pos_z").get_parameter_value().get<double>();//10;

    broadcaster_.sendTransform(transform_);
}

void CupNode::initMarker() {
    marker_message_ = visualization_msgs::msg::Marker();

    marker_message_.header.frame_id = cup_link_;
    marker_message_.header.stamp = now();

    marker_message_.ns = "water_cup";
    marker_message_.text = "virtual cup";
    marker_message_.id = 0;

    marker_message_.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    marker_message_.action = visualization_msgs::msg::Marker::ADD;

    std::string package_share_directory = ament_index_cpp::get_package_share_directory("robot_simulation");
    std::string file_name = package_share_directory.append("/model/wor_sim_cup.stl");

    marker_message_.mesh_resource = "file://" + file_name;

    // .stl scaling from meters to millimeters.
    marker_message_.scale.x = 0.001;
    marker_message_.scale.y = 0.001;
    marker_message_.scale.z = 0.001;

    marker_message_.color.r = 1.0f;
    marker_message_.color.g = 1.0f;
    marker_message_.color.b = 1.0f;
    marker_message_.color.a = 1.0f;

    marker_message_.pose.position.x = transform_.transform.translation.x;
    marker_message_.pose.position.y = transform_.transform.translation.y;
    marker_message_.pose.position.z = transform_.transform.translation.z;

    marker_message_.pose.orientation.x = 0.0;
    marker_message_.pose.orientation.y = 0.0;
    marker_message_.pose.orientation.z = 0.0;
    marker_message_.pose.orientation.w = 1.0;

    marker_message_.lifetime = builtin_interfaces::msg::Duration();
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CupNode>());
    rclcpp::shutdown();
    return 0;
}
