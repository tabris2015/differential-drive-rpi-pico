//
// Created by pepe on 16/6/21.
//

#ifndef DIFF_DRIVE_SERVO_H
#define DIFF_DRIVE_SERVO_H
#include "pico/stdlib.h"
#include "hardware/pwm.h"

class Servo {
public:
    Servo(uint gpio_pin, uint16_t min_us, uint16_t max_us);
    void write_duty_us(uint16_t us);
    void write_angle(uint angle);

private:
    uint _slice_num;
    uint _channel;
    uint _pin;
    uint16_t _offset_us;
    uint16_t _multiplier_us;
};


#endif //DIFF_DRIVE_SERVO_H
