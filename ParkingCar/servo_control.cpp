#include "servo_control.h"
#include "driver/ledc.h"
#include <Arduino.h>  // Required for pinMode


#define SERVO_RESOLUTION_BITS LEDC_TIMER_16_BIT
#define SERVO_TIMER_NUM       (ledc_timer_t)SERVO_TIMER
#define SERVO_CHANNEL_NUM     (ledc_channel_t)SERVO_CHANNEL

void servo_init(void) {
  // 1) configure the LEDC timer
  ledc_timer_config_t timer_cfg;
  timer_cfg.speed_mode      = LEDC_LOW_SPEED_MODE;
  timer_cfg.duty_resolution = SERVO_RESOLUTION_BITS;
  timer_cfg.timer_num       = SERVO_TIMER_NUM;
  timer_cfg.freq_hz         = SERVO_FREQ;
  timer_cfg.clk_cfg         = LEDC_AUTO_CLK;
  ledc_timer_config(&timer_cfg);

  // 2) configure the LEDC channel
  ledc_channel_config_t ch_cfg;
  ch_cfg.speed_mode = LEDC_LOW_SPEED_MODE;
  ch_cfg.channel    = SERVO_CHANNEL_NUM;
  ch_cfg.timer_sel  = SERVO_TIMER_NUM;
  ch_cfg.duty       = 0;
  ch_cfg.hpoint     = 0;
  ch_cfg.gpio_num   = SERVO_PIN;
  ledc_channel_config(&ch_cfg);

  // 3) move to neutral
  servo_set_angle(SERVO_FORWARD);
}
