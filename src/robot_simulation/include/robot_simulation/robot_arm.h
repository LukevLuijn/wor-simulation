//
// Created by luke on 25-10-22.
//

#ifndef WOR_SIMULATION_ROBOT_ARM_H
#define WOR_SIMULATION_ROBOT_ARM_H

#include <array>
#include <chrono>


typedef std::chrono::system_clock::time_point TimePoint;
typedef std::chrono::steady_clock::duration Duration;

struct Servo {
    double max_radians_per_sec_;

    double min_position_; // in radians
    double max_position_; // in radians

    uint16_t pwm_max_limit_;
    uint16_t pwm_min_limit_;

    TimePoint start_time_ = std::chrono::system_clock::now();
    Duration move_duration_ = std::chrono::milliseconds(1);
    bool is_moving_ = false;

    double previous_position_ = 0; // in radians
    double current_position_ = 0; // in radians
    double target_position_ = 0; // in radians
};

class RobotArm {
public:
    static RobotArm &get() {
        static RobotArm instance;
        return instance;
    }

public:
    void setTargetPosition(uint8_t index, uint16_t PWMValue);

    void setMoveDuration(uint8_t index, uint16_t duration);

    void activateLink(uint8_t index);

    void deactivateLink(uint8_t index);

    void stopRobot();

    std::string toString() const;

private:
    static constexpr uint8_t N_SERVOS = 6;
    std::array<Servo, N_SERVOS> links_;

    const std::array<std::string, N_SERVOS> servo_names_ = {
            "base", "shoulder", "elbow", "wrist", "gripper", "hand"};

protected:
    RobotArm();

    virtual ~RobotArm() = default;
};


#endif //WOR_SIMULATION_ROBOT_ARM_H
