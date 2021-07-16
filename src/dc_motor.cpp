//
// Created by pepe on 19/6/21.
//

#include "dc_motor.h"

DCMotor::DCMotor(uint enA_pin, uint enB_pin, uint pwm_pin)
:_enA_pin(enA_pin), _enB_pin(enB_pin), _pwm_pin(pwm_pin)
{
    // init PWM
    _slice_num = pwm_gpio_to_slice_num(_pwm_pin);
    _channel = pwm_gpio_to_channel(_pwm_pin);
    gpio_set_function(_pwm_pin, GPIO_FUNC_PWM);
    pwm_set_wrap(_slice_num, TOP);
    pwm_set_chan_level(_slice_num, _channel, 0);

    // init GPIO
    gpio_init(_enA_pin);
    gpio_init(_enB_pin);
    gpio_set_dir(_enA_pin, GPIO_OUT);
    gpio_set_dir(_enB_pin, GPIO_OUT);
    gpio_put(_enA_pin, false);
    gpio_put(_enB_pin, false);

    //
    pwm_set_enabled(_slice_num, true);
}

void DCMotor::write_int16(int16_t pwm)
{
    if(pwm < 0)
    {
        gpio_put(_enA_pin, true);
        gpio_put(_enB_pin, false);
        pwm_set_chan_level(_slice_num, _channel, abs(pwm));
    } else
    {
        gpio_put(_enA_pin, false);
        gpio_put(_enB_pin, true);
        pwm_set_chan_level(_slice_num, _channel, pwm);
    }
    pwm_set_enabled(_slice_num, true);
}

void DCMotor::write(float duty_cycle)
{
    if(duty_cycle > 1.0f) duty_cycle = 1.0f;
    if(duty_cycle < -1.0f) duty_cycle = -1.0f;
    write_int16((int16_t)(duty_cycle * TOP));
}



