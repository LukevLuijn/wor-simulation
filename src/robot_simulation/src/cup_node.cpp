#include "cup_node.h"

#include <ament_index_cpp/get_package_share_directory.hpp>

CupNode::CupNode()
        : Node("cup_node"), velocity_(0.0), cup_picked_up_(false), buffer_(get_clock()), listener_(buffer_),
          broadcaster_(this) {

    RCLCPP_INFO(this->get_logger(), "Hello world from cup node!"); // TODO for testing

    this->declare_parameter<double>("pos_x", 0.0);
    this->declare_parameter<double>("pos_y", 0.0);
    this->declare_parameter<double>("pos_z", 0.0);

    this->declare_parameter<std::string>("sim_link_name", "sim_link"); // world
    this->declare_parameter<std::string>("bot_link_name", "bot_link"); // robot
    this->declare_parameter<std::string>("cup_link_name", "cup_link"); // cup

    base_timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&CupNode::baseTimerCallback, this));
    speed_timer_ = this->create_wall_timer(std::chrono::milliseconds(200),
                                           std::bind(&CupNode::speedTimerCallback, this));
    cup_pickup_sub_ = this->create_subscription<CupPickup>("sim/arm/cup_pickup", 10,
                                                           std::bind(&CupNode::cupPickupCallback, this,
                                                                     std::placeholders::_1));

    marker_pub_ = create_publisher<Marker>("sim/cup/marker", 10);
    position_pub_ = create_publisher<Position>("sim/cup/position", 10);
    speed_pub_ = create_publisher<Speed>("sim/cup/speed", 10);

    sim_link_ = "base_link";//this->get_parameter("sim_link_name").get_parameter_value().get<std::string>();//"sim_link";
    bot_link_ = this->get_parameter("bot_link_name").get_parameter_value().get<std::string>();//"bot_link";
    cup_link_ = this->get_parameter("cup_link_name").get_parameter_value().get<std::string>();//"cup_link";

    initTransform();
    initMarker();

    default_color_.r = 1.0f;
    picked_up_color_.r = 0.0f;
    default_color_.g = picked_up_color_.g = 1.0f;
    default_color_.b = picked_up_color_.b = 1.0f;
    default_color_.a = picked_up_color_.a = 1.0f;
}

void CupNode::baseTimerCallback() {

    updateTransform();
    updateMarker();
    updateTopics();

    updatePosition();
}

void CupNode::speedTimerCallback() {
    geometry_msgs::msg::TransformStamped tf;
    tf = buffer_.lookupTransform(cup_link_, "sim_link", rclcpp::Time(0));

    double time = timeInSeconds(now() - tf.header.stamp);

    speed_message_.x = std::abs(tf.transform.translation.x - previous_tf_.transform.translation.x) / time;
    speed_message_.y = std::abs(tf.transform.translation.y - previous_tf_.transform.translation.y) / time;
    speed_message_.z = std::abs(tf.transform.translation.z - previous_tf_.transform.translation.z) / time;

    previous_tf_ = tf;
    speed_pub_->publish(speed_message_);
}

void CupNode::cupPickupCallback(const CupPickup::SharedPtr message) {

    cup_picked_up_ = message->state; // == CupPickup::HOLDING

    std::string targetFrameID = (cup_picked_up_) ? "hand" : sim_link_;

    Transform newTransform = buffer_.lookupTransform(targetFrameID, cup_link_, rclcpp::Time(0));
    transform_.transform = newTransform.transform;
    transform_.header.frame_id = targetFrameID;

    if (cup_picked_up_) {
        RCLCPP_INFO(this->get_logger(), "cup is picked up!");
    }
}

void CupNode::updateMarker() {
    marker_message_.color = (cup_picked_up_) ? picked_up_color_ : default_color_;
    marker_message_.header.stamp = now();
}

void CupNode::updateTransform() {
    if (!cup_picked_up_) {
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
        }
    }
    transform_.header.stamp = now();
}

void CupNode::updatePosition()
{
    geometry_msgs::msg::TransformStamped tf;
    tf = buffer_.lookupTransform(cup_link_, sim_link_, rclcpp::Time(0));

    simulation_msgs::msg::Position position;
    position.x = tf.transform.translation.x;
    position.y = tf.transform.translation.y;
    position.z = tf.transform.translation.z;

    position_pub_->publish(position);
}

void CupNode::updateTopics() {
    marker_pub_->publish(marker_message_);
    broadcaster_.sendTransform(transform_);
}

void CupNode::initTransform() {
    transform_.header.frame_id = sim_link_;
    transform_.child_frame_id = cup_link_;
    transform_.header.stamp = now();

    transform_.transform.translation.x = this->get_parameter("pos_x").get_parameter_value().get<double>();
    transform_.transform.translation.y = this->get_parameter("pos_y").get_parameter_value().get<double>();
    transform_.transform.translation.z = this->get_parameter("pos_z").get_parameter_value().get<double>();

    broadcaster_.sendTransform(transform_);
    previous_tf_ = transform_;
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

    marker_message_.color = default_color_;

    marker_message_.pose.position.x = 0.0;
    marker_message_.pose.position.y = 0.0;
    marker_message_.pose.position.z = 0.0;

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
