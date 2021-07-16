//
// Created by pepe on 19/6/21.
//

#ifndef DIFF_DRIVE_ENCODER_H
#define DIFF_DRIVE_ENCODER_H

#include "pico/stdlib.h"

#define DIR_NONE 0x0
#define DIR_CW 0x10
#define DIR_CCW 0x20

#define PREV_MASK 0x1
#define CURR_MASK 0x2
#define INVALID_MASK 0x3

#define ENC_MASK ((0x01 << ENC_A_PIN) | (0x01 << ENC_B_PIN))        // encoder channels should be together



class Encoder {
public:
    Encoder(uint A_pin, uint B_pin);
    void set_pulses(int32_t new_pulses);
    int32_t  get_pulses(void);

private:
    uint32_t _state;
    uint _A_pin;
    uint _B_pin;
    int32_t _pulses;
};


#endif //DIFF_DRIVE_ENCODER_H
