//
// Created by luke on 24-10-22.
//

#include "arm_node.h"
#include "robot_arm.h"

#include "string_utils.h"

#include "geometry_msgs/msg/transform_stamped.hpp"

ArmNode::ArmNode() : Node("arm_node"), broadcaster_(this) {
    RCLCPP_INFO(this->get_logger(), "Hello world from arm node!"); // TODO for testing

    this->declare_parameter<double>("pos_x", 0.0);
    this->declare_parameter<double>("pos_y", 0.0);
    this->declare_parameter<double>("pos_z", 0.0);

    this->declare_parameter<std::string>("sim_link_name", "sim_link"); // world
    this->declare_parameter<std::string>("bot_link_name", "bot_link"); // robot
    this->declare_parameter<std::string>("cup_link_name", "cup_link"); // cup

    timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&ArmNode::timerCallback, this));
    command_sub_ = this->create_subscription<Command>("sim/controller/command", 10,
                                                      std::bind(&ArmNode::commandCallback, this,
                                                                std::placeholders::_1));

    joint_state_pub_ = create_publisher<JointState>(/*sim/arm/*/"joint_states", 10);

    sim_link_ = this->get_parameter("sim_link_name").get_parameter_value().get<std::string>();//"sim_link";
    bot_link_ = this->get_parameter("bot_link_name").get_parameter_value().get<std::string>();//"bot_link";
    cup_link_ = this->get_parameter("cup_link_name").get_parameter_value().get<std::string>();//"cup_link";

    initJointState();
}

void ArmNode::timerCallback() {
    // TODO calculate current positions.

    updateTransform();
    updateJointState();

    RobotArm::get().updateRobot();


    /**
      * if gripper is closing && gripper position == cup position
      *
      *  arm is holding cup
      *
      * if gripper is opening && arm is holding cup
      *
      *  cup is released && arm is not holding cup
      */


//
//    if (!is_holding_cup_
//        && RobotArm::get().getGripperState() == RobotArm::GripperState_e::CLOSED
//        && RobotArm::get().getGripperPosition() == cupPosition)
//    {
//        is_holding_cup_ == true;
//
//        // publish to cup.
//    }
//    else if (is_holding_cup_
//    && RobotArm::get(). getGripperState() == RobotArm::GripperState_e::OPENED)
//    {
//        is_holding_cup_ = false;
//
//        // publish to cup.
//    }

}

//#0P500S5000#1P1833S5000#2P2000S5000#3P500S5000#4P2500S5000#5P500S5000      MAX
//#0P2500S5000#1P500S5000#2P500S5000#3P2000S5000#4P500S5000#5P2500S5000    MIN


//ros2 topic pub --once /sim/controller/command simulation_msgs/msg/Command "{command: '#0P500S5000#1P1833S5000#2P2000S5000#3P500S5000#4P2500S5000#5P500S5000'}"
//ros2 topic pub --once /sim/controller/command simulation_msgs/msg/Command "{command: '#0P2500S5000#1P500S5000#2P500S5000#3P2000S5000#4P500S5000#5P2500S5000'}"

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

                RCLCPP_INFO(this->get_logger(), "new command, moving robot");
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
void ArmNode::updateJointState()
{
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

void ArmNode::updateTransform()
{
    geometry_msgs::msg::TransformStamped transform;

    transform.header.stamp = now();
    transform.transform.translation.x = this->get_parameter("pos_x").get_parameter_value().get<double>();
    transform.transform.translation.y = this->get_parameter("pos_y").get_parameter_value().get<double>();
    transform.transform.translation.z = this->get_parameter("pos_z").get_parameter_value().get<double>();

    transform.header.frame_id = sim_link_;//"world_link";
    transform.child_frame_id = "base_link";

    broadcaster_.sendTransform(transform);
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