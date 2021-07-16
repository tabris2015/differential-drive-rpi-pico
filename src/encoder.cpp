//
// Created by pepe on 19/6/21.
//

#include "encoder.h"

Encoder::Encoder(uint A_pin, uint B_pin)
:_A_pin(A_pin), _B_pin(B_pin)
{
    gpio_init(_A_pin);
    gpio_init(_B_pin);
    gpio_pull_up(_A_pin);
    gpio_pull_up(_B_pin);


}
