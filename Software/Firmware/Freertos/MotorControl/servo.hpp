#ifndef __SERVO_HPP
#define __SERVO_HPP

class Servo;

#include <board.h>
#include <autogen/interfaces.hpp>

// Anonymous enum for defining the motor type
enum ServoMotorType {
    SERVO_MOTOR_TYPE_NONE = 0,
    SERVO_MOTOR_TYPE_CONTINUOUS = 1
};

class Servo : public ODriveIntf::ServoIntf {
    public:
    struct Config_t {
        uint32_t servo_motor_type = SERVO_MOTOR_TYPE_NONE;
        float servo_angle_min = 0.0f;
        float servo_angle_max = 180.0f;
    };

    Servo(int servo_num, TIM_HandleTypeDef* timer, uint32_t channel, Stm32Gpio gpio);

    Error error_ = ERROR_NONE;
    Config_t config_;

    float angle_ = 0.0f;
    float velocity_ = 0.0f;

    int servo_num_;
    TIM_HandleTypeDef* const timer_;
    uint32_t channel_;
    Stm32Gpio gpio_;

    bool set_angle(float angle);
};


#endif // __SERVO_HPP
