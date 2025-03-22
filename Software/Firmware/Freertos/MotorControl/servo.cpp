#include "servo.hpp"

#define PWM_MIN_PULSE 500  // 最小脉宽（单位：微秒），对应 0°
#define PWM_MAX_PULSE 2500 // 最大脉宽（单位：微秒），对应 180°

Servo::Servo(int servo_num, TIM_HandleTypeDef* timer, uint32_t channel, Stm32Gpio gpio) : 
        servo_num_(servo_num), timer_(timer), channel_(channel), gpio_(gpio)
{
}

bool Servo::set_angle(float angle) {
    if (angle < config_.servo_angle_min) angle = config_.servo_angle_min;
    if (angle > config_.servo_angle_max) angle = config_.servo_angle_max;

    angle_ = angle;
    
    // 计算占空比，对应脉宽范围：500us - 2500us
    uint16_t pulse = PWM_MIN_PULSE + (angle * (PWM_MAX_PULSE - PWM_MIN_PULSE) / (config_.servo_angle_max - config_.servo_angle_min));

    // 计算 PWM 寄存器值，假设时钟频率和 prescaler 已经设置好
    uint32_t pwm_period = __HAL_TIM_GET_AUTORELOAD(timer_) + 1;
    uint32_t pulse_value = (pulse * pwm_period) / 20000; // 20000us 是 50Hz 的周期

    // 设置 PWM 输出
    __HAL_TIM_SET_COMPARE(timer_, channel_, pulse_value);

    return true;
}
