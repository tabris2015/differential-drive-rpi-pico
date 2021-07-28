//
// Created by pepe on 14/7/21.
//

#include "robot.h"

Robot::Robot(
        float kp,
        float kd,
        float ki,
        uint32_t sample_time_ms,
        uint status_led_pin,
        RobotPins pins
        )
:
_kp(kp), _kd(kd), _ki(ki), _sample_time_ms(sample_time_ms),
_l_input(0.0f), _l_output(0.0f), _l_setpoint(0.0f),
_r_input(0.0f), _r_output(0.0f), _r_setpoint(0.0f),
_l_motor(pins.left.en_a, pins.left.en_b, pins.left.pwm),
_r_motor(pins.right.en_a, pins.right.en_b, pins.right.pwm),
_l_pid(&_l_input, &_l_output, &_l_setpoint, kp, ki, kd, sample_time_ms),
_r_pid(&_r_input, &_r_output, &_r_setpoint, kp, ki, kd, sample_time_ms),
_status_led_pin(status_led_pin)
{
    _l_motor.write(0.0f);
    _r_motor.write(0.0f);
    _l_pid.set_output_limits(-1.0f, 1.0f);
    _r_pid.set_output_limits(-1.0f, 1.0f);
    _l_setpoint = 0;
    _r_setpoint = 0;
    _pid_rate = float(sample_time_ms) / 1000.0f;
    initPins();
}

void Robot::updatePid(uint32_t l_encoder_ticks, uint32_t r_encoder_ticks)
{
    int32_t l_ticks = l_encoder_ticks;
    int32_t r_ticks = r_encoder_ticks;

    _state.l_position = (2.0 * M_PI) * l_ticks / ROBOT_MOTOR_PPR;
    _state.r_position = (2.0 * M_PI) * r_ticks / ROBOT_MOTOR_PPR;

    int32_t dl_ticks = l_ticks - _state.l_ticks;
    int32_t dr_ticks = r_ticks - _state.r_ticks;

    // update odometry
    updateOdometry(dl_ticks, dr_ticks);

    _state.l_ref_speed = _l_setpoint;
    _state.r_ref_speed = _r_setpoint;

    _state.l_speed = (2.0 * M_PI) * dl_ticks / (ROBOT_MOTOR_PPR * _pid_rate);
    _state.r_speed = (2.0 * M_PI) * dr_ticks / (ROBOT_MOTOR_PPR * _pid_rate);

    _odom.v = (ROBOT_WHEEL_RADIUS / 2.0f) * (_state.l_speed + _state.r_speed);
    _odom.w = (ROBOT_WHEEL_RADIUS / ROBOT_WHEEL_SEPARATION) * (_state.r_speed - _state.l_speed);

    _l_input = _state.l_speed;
    _r_input = _state.r_speed;

    _l_pid.compute();
    _r_pid.compute();

    _state.l_effort = _l_output;
    _state.r_effort = _r_output;

    _l_motor.write(_state.l_effort);
    _r_motor.write(_state.r_effort);

    _state.l_ticks = l_ticks;
    _state.r_ticks = r_ticks;
}

void Robot::updateOdometry(int32_t dl_ticks, int32_t dr_ticks) {
    float delta_l = (2 * M_PI * ROBOT_WHEEL_RADIUS * dl_ticks) / ROBOT_MOTOR_PPR;
    float delta_r = (2 * M_PI * ROBOT_WHEEL_RADIUS * dr_ticks) / ROBOT_MOTOR_PPR;
    float delta_center = (delta_l + delta_r) / 2;

    _odom.x_pos += delta_center * cosf(_odom.theta);
    _odom.y_pos += delta_center * sinf(_odom.theta);
    _odom.theta += (delta_r - delta_l) / ROBOT_WHEEL_SEPARATION;
    _odom.v = _linear;
    _odom.w = _angular;
}

void Robot::setWheels(float left_speed, float right_speed)
{
    _l_setpoint = left_speed;
    _r_setpoint = right_speed;
}

void Robot::setUnicycle(float v, float w)
{
    // limit values
    if(v > ROBOT_MAX_LINEAR_M_S) v = ROBOT_MAX_LINEAR_M_S;
    if(v < ROBOT_MIN_LINEAR_M_S) v = ROBOT_MIN_LINEAR_M_S;
    if(w > ROBOT_MAX_ANGULAR_R_S) w = ROBOT_MAX_ANGULAR_R_S;
    if(w < ROBOT_MIN_ANGULAR_R_S) w = ROBOT_MIN_ANGULAR_R_S;

    float v_l = (2 * v - w * ROBOT_WHEEL_SEPARATION) / (2 * ROBOT_WHEEL_RADIUS);
    float v_r = (2 * v + w * ROBOT_WHEEL_SEPARATION) / (2 * ROBOT_WHEEL_RADIUS);

    _linear = v;
    _angular = w;
    setWheels(v_l, v_r);
}

void Robot::initPins()
{
    gpio_init(_status_led_pin);
    gpio_set_dir(_status_led_pin, GPIO_OUT);
}

RobotState Robot::getState() {
    return _state;
}

RobotOdometry Robot::getOdometry() {
    return _odom;
}



