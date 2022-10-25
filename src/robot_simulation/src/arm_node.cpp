//
// Created by luke on 24-10-22.
//

#include "arm_node.h"
#include "robot_arm.h"

#include "string_utils.h"

ArmNode::ArmNode() : Node("arm_node") {
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

    sim_link_ = this->get_parameter("sim_link_name").get_parameter_value().get<std::string>();//"sim_link";
    bot_link_ = this->get_parameter("bot_link_name").get_parameter_value().get<std::string>();//"bot_link";
    cup_link_ = this->get_parameter("cup_link_name").get_parameter_value().get<std::string>();//"cup_link";
}

void ArmNode::timerCallback() {
    // TODO calculate current positions.
}

//#0P1000S1000#1P1200S1000#2P1590S1000#3P1500S1000#4P2500S1000#5P1500S1000
//ros2 topic pub --once /sim/controller/command simulation_msgs/msg/Command "{command: '#0P1000S60'}"
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

    // TODO remove
    std::cout << "\nNEW COMMAND HANDLED!!!\n" << std::endl;
    std::cout << RobotArm::get().toString() << std::endl;
}

bool ArmNode::parseCommandString(const std::string &command, std::vector<ServoCommand> &buffer) const {
    const char POSITION_CHAR = 'P', DURATION_CHAR = 'S';

    std::vector<std::string> commandStrings = Utils::StringUtils::divide(command, '#');
    std::vector<ServoCommand> commands;

    for (const std::string &str: commandStrings) {
        try {
            if (!str.empty()) {
                std::string indexString = Utils::StringUtils::divide(str, POSITION_CHAR).at(0);
                std::string positionString = Utils::StringUtils::divide(str, POSITION_CHAR, DURATION_CHAR).at(0);
                positionString = positionString.substr(1, positionString.size());
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

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmNode>());
    rclcpp::shutdown();
    return 0;
}