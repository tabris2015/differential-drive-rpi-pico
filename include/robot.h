//
// Created by pepe on 14/7/21.
//

#ifndef DIFF_DRIVE_ROBOT_H
#define DIFF_DRIVE_ROBOT_H

#include "pico/stdlib.h"
#include "math.h"
#include "pid_controller.h"
#include "dc_motor.h"

#define L_MOTOR_MIN_SPEED -1.0f
#define L_MOTOR_MAX_SPEED 1.0f
#define R_MOTOR_MIN_SPEED -1.0f
#define R_MOTOR_MAX_SPEED 1.0f
#define ROBOT_MOTOR_PPR 1496.0f
#define ROBOT_WHEEL_RADIUS 0.0325f
#define ROBOT_WHEEL_SEPARATION 0.17f

struct MotorPins
{
    uint pwm;
    uint en_a;
    uint en_b;
};

struct RobotPins
{
    MotorPins left;
    MotorPins right;
};

struct RobotState
{
    int32_t l_ticks;
    int32_t r_ticks;
    float l_position;
    float r_position;
    float l_speed;
    float r_speed;
    float l_effort;
    float r_effort;
    float l_ref_speed;
    float r_ref_speed;
};

struct RobotOdometry
{
    float x_pos;
    float y_pos;
    float theta;
    float v;
    float w;
};

class Robot{
public:
    Robot(
            float kp,
            float kd,
            float ki,
            uint32_t sample_time_ms,
            uint status_led_pin,
            RobotPins pins
            );
    void start();
    void setWheels(float left_speed, float right_speed);
    void setUnicycle(float v, float w);
    RobotState getState();
    RobotOdometry getOdometry();
    void setPidTunings(float kp, float kd, float ki);
    void updatePid(uint32_t l_encoder_ticks, uint32_t r_encoder_ticks);

private:
    float _kp;
    float _kd;
    float _ki;
    float _pid_rate;
    uint32_t _sample_time_ms;
    float _l_input;
    float _l_output;
    float _l_setpoint;
    float _r_input;
    float _r_output;
    float _r_setpoint;

    DCMotor _l_motor;
    DCMotor _r_motor;
    PID _l_pid;
    PID _r_pid;
    uint _status_led_pin;
    RobotState _state;
    RobotOdometry _odom;

    void controlLoop();
    void updateOdometry(int32_t dl_ticks, int32_t dr_ticks);
    void initPins();
};


#endif //DIFF_DRIVE_ROBOT_H
