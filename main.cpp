#include "stdio.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "robot.h"
#include "robot_pins.h"

#define MOTOR_PPR 1496.0f

#define PREV_MASK 0x1
#define CURR_MASK 0x2
#define INVALID_MASK 0x3

const uint32_t encoder1_mask = (0x01 << M1_ENC_A_PIN) | (0x01 << M1_ENC_B_PIN);
const uint32_t encoder2_mask = (0x01 << M2_ENC_A_PIN) | (0x01 << M2_ENC_B_PIN);
volatile int32_t encoder1_ticks = 0;
volatile uint32_t encoder1_state;

volatile int32_t encoder2_ticks = 0;
volatile uint32_t encoder2_state;

RobotPins robot_pins = {
        {
                M1_PWM_PIN,
                M1_ENA_PIN,
                M1_ENB_PIN
        },{
                M2_PWM_PIN,
                M2_ENA_PIN,
                M2_ENB_PIN
        }
};

float kp1 = 0.04;
float ki1 = 0.01;
float kd1 = 0;

uint32_t sample_time_ms = 20;

Robot robot(
        kp1, kd1, ki1,
        sample_time_ms,
        LED_PIN,
        robot_pins
        );

// encoder interrupt callback
void gpio_callback(uint gpio, uint32_t events)
{
    int32_t change1 = 0;
    int32_t change2 = 0;
    uint32_t new_state1 = ((gpio_get_all() & encoder1_mask) >> (M1_ENC_B_PIN > M1_ENC_A_PIN ? M1_ENC_A_PIN : M1_ENC_B_PIN)) & 0x3;
    uint32_t new_state2 = ((gpio_get_all() & encoder2_mask) >> (M2_ENC_B_PIN > M2_ENC_A_PIN ? M2_ENC_A_PIN : M2_ENC_B_PIN)) & 0x3;

    if(((new_state1 ^ encoder1_state) != INVALID_MASK) && (new_state1 != encoder1_state))
    {
        change1 = (encoder1_state & PREV_MASK) ^ ((new_state1 & CURR_MASK) >> 1);
        if(change1 == 0)
        {
            change1 = -1;
        }
        if(M1_ENC_INVERTED) change1 = -1 * change1;
        encoder1_ticks -= change1;
    }

    if(((new_state2 ^ encoder2_state) != INVALID_MASK) && (new_state2 != encoder2_state))
    {
        change2 = (encoder2_state & PREV_MASK) ^ ((new_state2 & CURR_MASK) >> 1);
        if(change2 == 0)
        {
            change2 = -1;
        }
        if(M2_ENC_INVERTED) change2 = -1 * change2;
        encoder2_ticks -= change2;
    }
    encoder1_state = new_state1;
    encoder2_state = new_state2;
}


void setup() {
    stdio_init_all();
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    // adc
    adc_init();
    adc_gpio_init(POT_PIN);
    adc_select_input(0);

    gpio_init(M1_ENC_A_PIN);
    gpio_pull_up(M1_ENC_A_PIN);
    gpio_init(M1_ENC_B_PIN);
    gpio_pull_up(M1_ENC_B_PIN);
    gpio_init(M2_ENC_A_PIN);
    gpio_pull_up(M2_ENC_A_PIN);
    gpio_init(M2_ENC_B_PIN);
    gpio_pull_up(M2_ENC_B_PIN);

    gpio_set_irq_enabled_with_callback(M1_ENC_A_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
    gpio_set_irq_enabled_with_callback(M1_ENC_B_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
    gpio_set_irq_enabled_with_callback(M2_ENC_A_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
    gpio_set_irq_enabled_with_callback(M2_ENC_B_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

    encoder1_state = ((gpio_get_all() & encoder1_mask) >> M1_ENC_A_PIN) & 0x3;
    encoder2_state = ((gpio_get_all() & encoder2_mask) >> M2_ENC_A_PIN) & 0x3;

}


void printState(float v, float w, RobotState state, RobotOdometry odometry)
{
    printf(
            "( %.2f, %.2f ) Setpoint: %.2f <=> %.2f \tSpeed: %.2f <=> %.2f \t Effort: %.2f <=> %.2f\r",
            v, w,
            state.l_ref_speed, state.r_ref_speed, state.l_speed, state.r_speed, state.l_effort, state.r_effort);

}

int main()
{
    setup();
    uint32_t i = 0;
    float linear = 0.1;
    float angular = 0;
    sleep_ms(1000);
    while (true)
    {
        gpio_put(LED_PIN, true);
        sleep_ms(5);
        gpio_put(LED_PIN, false);
        sleep_ms(15);

        robot.setUnicycle(linear, 0.0f);
        robot.updatePid(encoder1_ticks, encoder2_ticks);

        i++;
        if(i % 10 == 0)
        {
            printState(linear, angular, robot.getState(), robot.getOdometry());
        }
    }
}

