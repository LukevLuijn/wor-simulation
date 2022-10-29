//
// Created by luke on 24-10-22.
//

#include "arm_node.h"
#include "robot_arm.h"

#include "string_utils.h"
#include "simulation_msgs/msg/cup_pickup.hpp"

ArmNode::ArmNode() : Node("arm_node"), is_holding_cup_(false), buffer_(get_clock()), listener_(buffer_),
                     broadcaster_(this) {
    RCLCPP_INFO(this->get_logger(), "Hello world from arm node!"); // TODO for testing

    this->declare_parameter<double>("pos_x", 0.0);
    this->declare_parameter<double>("pos_y", 0.0);
    this->declare_parameter<double>("pos_z", 0.0);

    this->declare_parameter<std::string>("sim_link_name", "sim_link"); // world
    this->declare_parameter<std::string>("bot_link_name", "bot_link"); // robot
    this->declare_parameter<std::string>("cup_link_name", "cup_link"); // cup

    timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&ArmNode::timerCallback, this));
    command_sub_ = this->create_subscription<Command>("sim/controller/command", 10,
                                                      std::bind(&ArmNode::commandCallback, this,
                                                                std::placeholders::_1));

    joint_state_pub_ = create_publisher<JointState>("joint_states", 10);
    cup_pickup_pub_ = create_publisher<CupPickup>("sim/arm/cup_pickup", 10);

    sim_link_ = this->get_parameter("sim_link_name").get_parameter_value().get<std::string>();
    bot_link_ = this->get_parameter("bot_link_name").get_parameter_value().get<std::string>();
    cup_link_ = this->get_parameter("cup_link_name").get_parameter_value().get<std::string>();

    initJointState();
}

void ArmNode::timerCallback() {

    updateTransform();
    updateJointState();

    RobotArm::get().updateRobot();

    if (!is_holding_cup_ && RobotArm::get().getGripperState() == RobotArm::GripperState_e::CLOSED) {
        if (canPickupCup()) {
            is_holding_cup_ = simulation_msgs::msg::CupPickup::HOLDING;

            simulation_msgs::msg::CupPickup message;
            message.state = is_holding_cup_;
            cup_pickup_pub_->publish(message);
        }
    } else if (is_holding_cup_ && RobotArm::get().getGripperState() == RobotArm::GripperState_e::OPENED) {
        is_holding_cup_ = simulation_msgs::msg::CupPickup::RELEASED;

        simulation_msgs::msg::CupPickup message;
        message.state = is_holding_cup_;
        cup_pickup_pub_->publish(message);
    }
}

void ArmNode::commandCallback(const Command::SharedPtr command_msg) {
    std::vector<ServoCommand> commands;

    if (command_msg->command == "STOP") {
        RCLCPP_INFO(this->get_logger(), "STOP command detected, stopping robot.");
        RobotArm::get().stopRobot();
    } else if (parseCommandString(command_msg->command, commands)) {
        for (const ServoCommand &command: commands) {
            try {
                RobotArm::get().setTargetPosition(command.index_, command.position_);
                RobotArm::get().setMoveDuration(command.index_, command.duration_);
                RobotArm::get().activateLink(command.index_);
            }
            catch (std::invalid_argument &e) {
                RCLCPP_ERROR(this->get_logger(), "exception caught: %s", e.what());
                return; // ignore command
            }
            catch (...) {
                RCLCPP_ERROR(this->get_logger(), "undefined exception caught at: %s", __PRETTY_FUNCTION__);
                return; // ignore command
            }
        }
    }
    RCLCPP_INFO(this->get_logger(), "moving robot..");
}

bool ArmNode::parseCommandString(const std::string &command, std::vector<ServoCommand> &buffer) const {
    const char POSITION_CHAR = 'P', DURATION_CHAR = 'S';

    std::vector<std::string> commandStrings = Utils::StringUtils::divide(command, '#');
    std::vector<ServoCommand> commands;

    for (const std::string &str: commandStrings) {
        try {
            if (!str.empty()) {
                // retrieve index
                std::string indexString = Utils::StringUtils::divide(str, POSITION_CHAR).at(0);
                // retrieve position
                std::string positionString = Utils::StringUtils::divide(str, POSITION_CHAR, DURATION_CHAR).at(0);
                // remove index from position
                positionString = positionString.substr(1, positionString.size());
                // retrieve duration
                std::string durationString = Utils::StringUtils::divide(str, DURATION_CHAR).at(1);

                ServoCommand servoCommand = {
                        static_cast<uint8_t>(std::stoi(indexString)),
                        static_cast<uint16_t>(std::stoi(positionString)),
                        static_cast<uint16_t>(std::stoi(durationString))};

                commands.emplace_back(servoCommand);
            }
        }
        catch (std::out_of_range &e) {
            RCLCPP_ERROR(this->get_logger(), "exception caught %s", e.what());
            RCLCPP_DEBUG(this->get_logger(), "ill formed command string: %s in %s", str.c_str(),
                         command.c_str());
            return false; // ignore command
        }
        catch (std::invalid_argument &e) {
            RCLCPP_ERROR(this->get_logger(), "exception caught %s", e.what());
            RCLCPP_DEBUG(this->get_logger(), "ill formed command string: %s in %s", str.c_str(),
                         command.c_str());
            return false; // ignore command
        }
        catch (...) {
            RCLCPP_ERROR(this->get_logger(), "unknown exception caught at: %s", __PRETTY_FUNCTION__);
            RCLCPP_DEBUG(this->get_logger(), "ill formed command string: %s in %s", str.c_str(),
                         command.c_str());
            return false; // ignore command
        }
    }
    buffer = commands;
    return true;
}

void ArmNode::updateJointState() {
    joint_state_message_.header.stamp = now();
    joint_state_message_.position = {
            RobotArm::get().getCurrentPosition(0), // base
            RobotArm::get().getCurrentPosition(1), // shoulder
            RobotArm::get().getCurrentPosition(2), // elbow
            RobotArm::get().getCurrentPosition(3), // wrist
            RobotArm::get().getCurrentPosition(5), // hand
            RobotArm::get().getCurrentPosition(4), // gripper 01
            RobotArm::get().getCurrentPosition(4), // gripper 02
    };
    joint_state_pub_->publish(joint_state_message_);
}

void ArmNode::updateTransform() {
    transform_stamped_.header.frame_id = sim_link_;
    transform_stamped_.child_frame_id = "base_link";
    transform_stamped_.header.stamp = now();

    transform_stamped_.transform.translation.x = this->get_parameter("pos_x").get_parameter_value().get<double>();
    transform_stamped_.transform.translation.y = this->get_parameter("pos_y").get_parameter_value().get<double>();
    transform_stamped_.transform.translation.z = this->get_parameter("pos_z").get_parameter_value().get<double>();

    broadcaster_.sendTransform(transform_stamped_);
}

bool ArmNode::canPickupCup() {
    const double MAX_OFFSET = 0.04;

    Transform tfLeft = buffer_.lookupTransform(cup_link_, "gripper_left", rclcpp::Time(0));
    Transform tfRight = buffer_.lookupTransform(cup_link_, "gripper_right", rclcpp::Time(0));

    const double xLeft = tfLeft.transform.translation.x;
    const double xRight = tfRight.transform.translation.x;
    const double yLeft = tfLeft.transform.translation.y;
    const double yRight = tfRight.transform.translation.y;

    const double x = std::sqrt(std::pow(xLeft, 2) + std::pow(xRight, 2));
    const double y = std::sqrt(std::pow(yLeft, 2) + std::pow(yRight, 2));

    return (x <= MAX_OFFSET) && (y <= MAX_OFFSET);
}

void ArmNode::initJointState() {
    joint_state_message_.header.stamp = now();
    joint_state_message_.name = {
            "base_link2turret",
            "turret2upperarm",
            "upperarm2forearm",
            "forearm2wrist",
            "wrist2hand",
            "gripper_left2hand",
            "gripper_right2hand"};
    joint_state_message_.position = {
            RobotArm::get().getPreviousPosition(0), // base
            RobotArm::get().getPreviousPosition(1), // shoulder
            RobotArm::get().getPreviousPosition(2), // elbow
            RobotArm::get().getPreviousPosition(3), // wrist
            RobotArm::get().getPreviousPosition(5), // hand
            RobotArm::get().getPreviousPosition(4), // gripper 01
            RobotArm::get().getPreviousPosition(4), // gripper 02
    };
    joint_state_pub_->publish(joint_state_message_);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmNode>());
    rclcpp::shutdown();
    return 0;
}