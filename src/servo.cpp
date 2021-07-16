//
// Created by pepe on 16/6/21.
//

#include "../include/servo.h"

Servo::Servo(uint gpio_pin, uint16_t min_us, uint16_t max_us)
: _pin(gpio_pin)
{
    // init pin as PWM
    _slice_num = pwm_gpio_to_slice_num(_pin);
    _channel = pwm_gpio_to_channel(_pin);
    _offset_us = min_us;
    _multiplier_us = (max_us - min_us) / 180;
    gpio_set_function(_pin, GPIO_FUNC_PWM);
    // TODO: parametric divider and wrap for pwm
    pwm_set_clkdiv(_slice_num, 125.0);
    pwm_set_wrap(_slice_num, 20000);
    pwm_set_chan_level(_slice_num, _channel, 0);
    pwm_set_enabled(_slice_num, true);
}

void Servo::write_duty_us(uint16_t us) {
    pwm_set_chan_level(_slice_num, _channel, us);
    pwm_set_enabled(_slice_num, true);
}

void Servo::write_angle(uint angle) {
    if(angle > 180 || angle < 0) return;
    write_duty_us((angle * _multiplier_us) + _offset_us);
}




