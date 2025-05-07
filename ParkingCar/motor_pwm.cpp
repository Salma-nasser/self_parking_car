#include "motor_pwm.h"
#include <Arduino.h>
#include "driver/ledc.h"
#include "esp_err.h"

void motor_pwm_init(uint8_t pwm_pin, uint8_t channel, uint32_t freq, uint8_t resolution) {
  // Configure LEDC timer
  ledc_timer_config_t timer;
  timer.speed_mode = LEDC_LOW_SPEED_MODE;
  timer.duty_resolution = (ledc_timer_bit_t)resolution;
  timer.timer_num = LEDC_TIMER_0;
  timer.freq_hz = freq;
  timer.clk_cfg = LEDC_AUTO_CLK;
  ledc_timer_config(&timer);

  // Configure LEDC channel
  ledc_channel_config_t ledc_channel;
  ledc_channel.channel = (ledc_channel_t)channel;
  ledc_channel.duty = 0;
  ledc_channel.gpio_num = pwm_pin;
  ledc_channel.speed_mode = LEDC_LOW_SPEED_MODE;
  ledc_channel.hpoint = 0;
  ledc_channel.timer_sel = LEDC_TIMER_0;
  ledc_channel.intr_type = LEDC_INTR_DISABLE; // Required in some versions

  ledc_channel_config(&ledc_channel);
}

void motor_set_speed(uint8_t channel, uint8_t speed_percent) {
  uint32_t duty_max = (1 << LEDC_TIMER_8_BIT) - 1;
  uint32_t duty = (speed_percent * duty_max) / 100;

  ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)channel, duty);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)channel);
}
