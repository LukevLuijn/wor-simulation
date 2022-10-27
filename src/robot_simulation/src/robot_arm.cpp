//
// Created by luke on 25-10-22.
//

#include <iomanip>
#include "robot_arm.h"
#include "math_utils.h"

RobotArm::RobotArm()
        : links_(std::array<Servo, 6>{
        Servo{Utils::MathUtils::toRadians(333.333),
              Utils::MathUtils::toRadians(-90),
              Utils::MathUtils::toRadians(90),
              500,
              2500},
        Servo{Utils::MathUtils::toRadians(428.571),
              Utils::MathUtils::toRadians(-30),
              Utils::MathUtils::toRadians(90),
              1833,
              500},
//        Servo{Utils::MathUtils::toRadians(260.869),
//              Utils::MathUtils::toRadians(0),
//              Utils::MathUtils::toRadians(135),
//              500,
//              2000},
        Servo{Utils::MathUtils::toRadians(260.869),
              Utils::MathUtils::toRadians(135),
              Utils::MathUtils::toRadians(0),
              2000,
              500},
        Servo{Utils::MathUtils::toRadians(300.000),
              Utils::MathUtils::toRadians(90),
              Utils::MathUtils::toRadians(-90),
              500,
              2500},
        Servo{Utils::MathUtils::toRadians(2.292),
              Utils::MathUtils::toRadians(1.2),
              Utils::MathUtils::toRadians(1.2),
              2500,
              500},
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
}

void RobotArm::setTargetPosition(uint8_t index, uint16_t PWMValue) {
    if (index < N_SERVOS) {
        const double IN_MIN = links_[index].pwm_min_limit_;
        const double IN_MAX = links_[index].pwm_max_limit_;
        const double OUT_MIN = links_[index].min_position_;
        const double OUT_MAX = links_[index].max_position_;

        const double pos = Utils::MathUtils::map(PWMValue, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX);
        links_[index].target_position_ = pos;

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


void RobotArm::stopRobot() {
    for (std::size_t i = 0; i < links_.size(); ++i) {
        deactivateLink(i);
    }
}

double RobotArm::getCurrentPosition(uint8_t index) const {
    return getter(index, links_[index].current_position_);
}

double RobotArm::getPreviousPosition(uint8_t index) const {
    return getter(index, links_[index].previous_position_);
}

double RobotArm::getTargetPosition(uint8_t index) const {
    return getter(index, links_[index].target_position_);
}

std::string RobotArm::toString() const // TODO temp
{
    std::string robotString;

    uint8_t counter = 0;
    for (const Servo &link: links_) {
        std::string linkStr =
                "\nServo [" + std::to_string(static_cast<uint16_t>(counter)) + "]: " + servo_names_[counter];

        auto durationInMS = std::chrono::duration_cast<std::chrono::milliseconds>(link.move_duration_).count();
        std::string duration = std::to_string(durationInMS);

        const std::time_t tt = std::chrono::system_clock::to_time_t(link.start_time_);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&tt), "%X");
        std::string startTime = ss.str();

        linkStr.append("\n\tis moving?:\t" + std::string((link.is_moving_) ? "yes" : "no"));
        linkStr.append("\n\tstart time:\t" + startTime);
        linkStr.append("\n\tduration:\t" + duration + " ms");
        linkStr.append("\n\tprevious pos:\t" + std::to_string(Utils::MathUtils::toDegrees(link.previous_position_)) +
                       " degrees");
        linkStr.append("\n\tcurrent pos:\t" + std::to_string(Utils::MathUtils::toDegrees(link.current_position_)) +
                       " degrees");
        linkStr.append(
                "\n\ttarget pos:\t" + std::to_string(Utils::MathUtils::toDegrees(link.target_position_)) + " degrees");

        linkStr.append("\n\t---");
        // constants
        linkStr.append("\n\tdegrees p/sec:\t" + std::to_string(Utils::MathUtils::toDegrees(link.max_radians_per_sec_)) +
                       " degrees");
        linkStr.append(
                "\n\tmin position:\t" + std::to_string(Utils::MathUtils::toDegrees(link.min_position_)) + " degrees");
        linkStr.append(
                "\n\tmax position:\t" + std::to_string(Utils::MathUtils::toDegrees(link.max_position_)) + " degrees");
        linkStr.append("\n\tPWM limit max:\t" + std::to_string(link.pwm_max_limit_));
        linkStr.append("\n\tPWM limit min:\t" + std::to_string(link.pwm_min_limit_));

        ++counter;
        robotString.append(linkStr + "\n");
    }
    return robotString;
}

