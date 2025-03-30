#include "servo.hpp"

Servo::Servo(int servo_num, TIM_HandleTypeDef* timer, uint32_t channel, Stm32Gpio gpio) :
        servo_num_(servo_num), timer_(timer), channel_(channel), gpio_(gpio)
{
}

bool Servo::set_angle(float angle) {
    if (angle < config_.angle_min) angle = config_.angle_min;
    if (angle > config_.angle_max) angle = config_.angle_max;

    angle_ = angle;
    // 计算占空比值
    float pulse = config_.pulse_min + (angle * (config_.pulse_max - config_.pulse_min) / (config_.angle_max - config_.angle_min));
    // 设置 PWM 输出
    return set_pulse((int32_t)pulse);
}

bool Servo::set_pulse(float pulse)
{
    pulse_ = (int32_t)pulse;
    __HAL_TIM_SET_COMPARE(timer_, channel_, pulse_);
    return true;
}
