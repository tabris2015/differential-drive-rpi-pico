//
// Created by pepe on 21/6/21.
//

#include "pid_controller.h"

PID::PID(float *input, float *output, float *setpoint, float kp, float ki, float kd, uint32_t sample_time_ms)
: _my_input(input), _my_output(output), _my_setpoint(setpoint), _sample_time_ms(sample_time_ms), _out_sum(0), _last_input(0)
{
    set_output_limits(-1.0f, 1.0f);
    set_gains(kp, ki, kd);
}

void PID::set_sample_time(uint32_t new_sample_time_ms)
{
    float ratio = (float)new_sample_time_ms / _sample_time_ms;

    _ki *= ratio;
    _kd /= ratio;
    _sample_time_ms = new_sample_time_ms;
}

void PID::set_gains(float kp, float ki, float kd)
{
    if(kp < 0 || ki < 0 || kd < 0) return;

    float sample_time_s = (float)_sample_time_ms / 1000.0f;
    printf("pid sample time: %f [s]", sample_time_s);

    _kp = kp;
    _ki = ki * sample_time_s;
    _kd = kd / sample_time_s;
}

void PID::set_output_limits(float min, float max)
{
    if(min >= max) return;
    _out_min = min;
    _out_max = max;

    if(*_my_output > _out_max) *_my_output = _out_max;
    else if(*_my_output < _out_min) *_my_output = _out_min;

    if(_out_sum > _out_max) _out_sum = _out_max;
    else if(_out_sum < _out_min) _out_sum = _out_min;
}

void PID::compute(void)
{
    float input = *_my_input;
    float error = *_my_setpoint - input;

    float delta_input = input - _last_input;
    _out_sum += _ki * error;

    if(_out_sum > _out_max) _out_sum = _out_max;
    else if(_out_sum < _out_min) _out_sum = _out_min;

    float output = _kp * error + _out_sum - _kd * delta_input;

    if(output > _out_max) output = _out_max;
    else if(output < _out_min) output = _out_min;

    *_my_output = output;
    _last_input = input;
}


