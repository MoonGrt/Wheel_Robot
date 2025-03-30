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

// Anonymous enum for defining the servo init mode
enum ServoInitMode {
    SERVO_INIT_MODE_UNDEFINED = 0,
    SERVO_INIT_MODE_NONE = 1,
    SERVO_INIT_MODE_PULSE = 2,
    SERVO_INIT_MODE_ANGLE = 3,
};

class Servo : public ODriveIntf::ServoIntf {
   public:
    struct Config_t {
        uint32_t servo_motor_type = SERVO_MOTOR_TYPE_NONE;
        uint32_t servo_freq = 50;
        uint32_t init_mode = SERVO_INIT_MODE_NONE;
        float init_pulse = 0.0f;
        float init_angle = 0.0f;
        float pulse_min = 500.0f;
        float pulse_max = 2500.0f;
        float angle_min = 0.0f;
        float angle_max = 180.0f;
    };

    Servo(int servo_num, TIM_HandleTypeDef* timer, uint32_t channel, Stm32Gpio gpio);

    Config_t config_;
    Error error_ = ERROR_NONE;

    // ServoState ServoState_ = SERVO_STATE_IDLE;

    float angle_ = 0.0f;
    uint32_t pulse_ = 0;
    // float velocity_ = 0.0f;

    int servo_num_;
    TIM_HandleTypeDef* const timer_;
    uint32_t channel_;
    Stm32Gpio gpio_;

    bool set_angle(float angle);
    bool set_pulse(float pulse);
};

#endif  // __SERVO_HPP
