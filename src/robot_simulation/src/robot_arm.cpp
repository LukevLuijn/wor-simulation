//
// Created by luke on 25-10-22.
//

#include <iomanip>
#include "robot_arm.h"
#include "math_utils.h"

// todo remove
#include "iostream"

RobotArm::RobotArm()
        : current_gripper_state_(GripperState_e::OPENED), links_(std::array<Servo, 6>{
        // BASE
        Servo{Utils::MathUtils::toRadians(333.333),
              Utils::MathUtils::toRadians(-90),
              Utils::MathUtils::toRadians(90),
              500,
              2500},
        // SHOULDER
        Servo{Utils::MathUtils::toRadians(428.571),
              Utils::MathUtils::toRadians(-30),
              Utils::MathUtils::toRadians(90),
              1833,
              500},
        // ELBOW
        Servo{Utils::MathUtils::toRadians(260.869),
              Utils::MathUtils::toRadians(-135.0),
              Utils::MathUtils::toRadians(0.0),
              500,
              2000,
        },
        // WRIST
        Servo{Utils::MathUtils::toRadians(300.000),
              Utils::MathUtils::toRadians(90),
              Utils::MathUtils::toRadians(-90),
              500,
              2500},
        // GRIPPER
        Servo{Utils::MathUtils::toRadians(2.292),
              Utils::MathUtils::toRadians(-1.2),
              Utils::MathUtils::toRadians(1.2),
              2500,
              500},
        // WRIST_ROTATE
        Servo{Utils::MathUtils::toRadians(375.000),
              Utils::MathUtils::toRadians(-90),
              Utils::MathUtils::toRadians(90),
              500,
              2500}}) {
}

void RobotArm::updateRobot() {
    auto durationToMS = [](const Duration &duration) -> double {
        return static_cast<double>(std::chrono::duration_cast<std::chrono::milliseconds>(duration).count());
    };

    for (Servo &link: links_) {
        if (link.is_moving_) {
            TimePoint currentTime = std::chrono::system_clock::now();
            // check if move duration has not passed.
            if (currentTime - link.start_time_ < link.move_duration_) { // still working on move.
                // get total time working on current move.
                const double timeMoving = durationToMS(currentTime - link.start_time_);
                // get total time for move.
                const double moveDuration = durationToMS(link.move_duration_);
                // get percentage of move based on time passed.
                const double percentageComplete = timeMoving / moveDuration;
                // get movement delta based on movement percentage.
                const double movementDelta = (link.target_position_ - link.previous_position_) * percentageComplete;
                // update position with the movement delta.
                link.current_position_ = link.previous_position_ + movementDelta;
            } else { // move is complete
                link.current_position_ = link.target_position_;
                deactivateLink(link);
            }
        }
    }
    // check gripper state
    checkGripperState();
}

void RobotArm::setTargetPosition(uint8_t index, uint16_t PWMValue) {
    if (index < N_SERVOS) {
        const double IN_MIN = links_[index].pwm_min_limit_;
        const double IN_MAX = links_[index].pwm_max_limit_;
        const double OUT_MIN = links_[index].min_position_;
        const double OUT_MAX = links_[index].max_position_;

        const double pos = Utils::MathUtils::map(PWMValue, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX);
        // axis two needs a 90 degree negative offset.
        links_[index].target_position_ = pos;// TODO (index == 2) ? pos - Utils::MathUtils::toRadians(90) : pos;
    } else {
        throw std::invalid_argument("servo index out of bounds: " + std::to_string(static_cast<uint16_t>(index)));
    }
}

void RobotArm::setTargetPosition(uint8_t index, double degrees) {
    if (index < N_SERVOS) {

        double radians = Utils::MathUtils::toRadians(degrees);

        if (Utils::MathUtils::between(links_[index].max_position_, links_[index].min_position_, radians)) {
            links_[index].target_position_ = Utils::MathUtils::toRadians(degrees);
        } else { // ignore
            throw std::invalid_argument(
                    "position out of range: " + std::to_string(static_cast<uint16_t>(index)) + ", " +
                    std::to_string(degrees));
        }
    } else {
        throw std::invalid_argument("servo index out of bounds: " + std::to_string(static_cast<uint16_t>(index)));
    }
}


void RobotArm::setMoveDuration(uint8_t index, uint16_t duration) {
    if (index < N_SERVOS) {
        links_[index].previous_position_ = links_[index].current_position_;

        double distanceToTarget = std::abs(links_[index].previous_position_ - links_[index].target_position_);
        double minimalDuration = distanceToTarget / (links_[index].max_radians_per_sec_ / 1000.0);
        auto moveDuration = static_cast<uint16_t>((minimalDuration < duration) ? duration : minimalDuration);

        links_[index].move_duration_ = std::chrono::milliseconds(moveDuration);
    } else {
        throw std::invalid_argument("servo index out of bounds: " + std::to_string(static_cast<uint16_t>(index)));
    }
}

void RobotArm::activateLink(uint8_t index) {
    if (index < N_SERVOS) {
        links_[index].start_time_ = std::chrono::system_clock::now();
        links_[index].is_moving_ = true;
    } else {
        throw std::invalid_argument("servo index out of bounds: " + std::to_string(static_cast<uint16_t>(index)));
    }
}

void RobotArm::deactivateLink(uint8_t index) {
    if (index < N_SERVOS) {
        deactivateLink(links_[index]);
    } else {
        throw std::invalid_argument("servo index out of bounds: " + std::to_string(static_cast<uint16_t>(index)));
    }
}

void RobotArm::deactivateLink(Servo &servo) {
    servo.target_position_ = servo.current_position_;
    servo.move_duration_ = std::chrono::milliseconds(0);
    servo.is_moving_ = false;
}

void RobotArm::checkGripperState() {
    const double CLOSED_OFFSET = 0.02;
    current_gripper_state_ = (links_[GRIPPER_INDEX].current_position_ >= CLOSED_OFFSET) ? GripperState_e::CLOSED
                                                                                        : GripperState_e::OPENED;
}

void RobotArm::stopRobot() {
    for (Servo &link: links_) {
        deactivateLink(link);
    }
}

RobotArm::GripperState_e RobotArm::getGripperState() const {
    return current_gripper_state_;
}

double RobotArm::getCurrentPosition(uint8_t index) const {
    double value = getter(index, links_[index].current_position_);
    value -= (index == 2) ? Utils::MathUtils::toRadians(90) : 0;
    return value;
}

double RobotArm::getPreviousPosition(uint8_t index) const {
    double value = getter(index, links_[index].previous_position_);
    value -= (index == 2) ? Utils::MathUtils::toRadians(90) : 0;
    return value;
}

double RobotArm::getTargetPosition(uint8_t index) const {
    double value = getter(index, links_[index].target_position_);
    value -= (index == 2) ? Utils::MathUtils::toRadians(90) : 0;
    return value;
}

