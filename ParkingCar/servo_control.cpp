// #include "servo_control.h"
// #include "driver/ledc.h"
// #include <Arduino.h>  // Required for pinMode


// #define SERVO_RESOLUTION_BITS LEDC_TIMER_16_BIT
// #define SERVO_TIMER_NUM       (ledc_timer_t)SERVO_TIMER
// #define SERVO_CHANNEL_NUM     (ledc_channel_t)SERVO_CHANNEL

// // Create servo object
// Servo myservo;

// void servo_set_angle(uint8_t angle) {
//   // Ensure angle is within valid range (0-180)
//   if (angle > 180) angle = 180;
//   // Write angle to servo
//   myservo.write(angle);
// }

// void servo_init(void) {
//   // 1) configure the LEDC timer
//   ledc_timer_config_t timer_cfg;
//   timer_cfg.speed_mode      = LEDC_LOW_SPEED_MODE;
//   timer_cfg.duty_resolution = SERVO_RESOLUTION_BITS;
//   timer_cfg.timer_num       = SERVO_TIMER_NUM;
//   timer_cfg.freq_hz         = SERVO_FREQ;
//   timer_cfg.clk_cfg         = LEDC_AUTO_CLK;
//   ledc_timer_config(&timer_cfg);

//   // 2) configure the LEDC channel
//   ledc_channel_config_t ch_cfg;
//   ch_cfg.speed_mode = LEDC_LOW_SPEED_MODE;
//   ch_cfg.channel    = SERVO_CHANNEL_NUM;
//   ch_cfg.timer_sel  = SERVO_TIMER_NUM;
//   ch_cfg.duty       = 0;
//   ch_cfg.hpoint     = 0;
//   ch_cfg.gpio_num   = SERVO_PIN;
//   ledc_channel_config(&ch_cfg);

//   // 3) move to neutral
//   servo_set_angle(SERVO_FORWARD);
// }
#include "servo_control.h"
#include "driver/ledc.h"
#include <Arduino.h>

#define SERVO_RESOLUTION_BITS LEDC_TIMER_16_BIT
#define SERVO_TIMER_NUM       (ledc_timer_t)SERVO_TIMER
#define SERVO_CHANNEL_NUM     (ledc_channel_t)SERVO_CHANNEL

// Convert angle to duty cycle
uint32_t angle_to_duty(uint8_t angle) {
    // Servo pulse width range: 500μs - 2500μs
    // For 16-bit resolution (0-65535)
    const uint32_t min_duty = 1638;  // 500μs/20ms * 65535
    const uint32_t max_duty = 8192;  // 2500μs/20ms * 65535
    return min_duty + (((max_duty - min_duty) * angle) / 180);
}

void servo_set_angle(uint8_t angle) {
    // Ensure angle is within valid range (0-180)
    if (angle > 180) angle = 180;
    
    // Convert angle to duty cycle and set it
    uint32_t duty = angle_to_duty(angle);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, SERVO_CHANNEL_NUM, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, SERVO_CHANNEL_NUM);
}

void servo_init(void) {
    // 1) configure the LEDC timer
    ledc_timer_config_t timer_cfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = SERVO_RESOLUTION_BITS,
        .timer_num = SERVO_TIMER_NUM,
        .freq_hz = SERVO_FREQ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_cfg);

    // 2) configure the LEDC channel
    ledc_channel_config_t ch_cfg = {
        .gpio_num = SERVO_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = SERVO_CHANNEL_NUM,
        .timer_sel = SERVO_TIMER_NUM,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&ch_cfg);

    // 3) move to neutral position
    servo_set_angle(SERVO_FORWARD);
}