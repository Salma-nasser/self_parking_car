#include "motor_pwm.h"

#include "driver/ledc.h"
#include "esp_err.h"

void motor_pwm_init(uint8_t pwm_pin, uint8_t channel, uint32_t freq, uint8_t resolution) {
  ledc_timer_config_t timer = {.speed_mode = LEDC_LOW_SPEED_MODE,
                               .timer_num = LEDC_TIMER_0,
                               .duty_resolution = resolution,
                               .freq_hz = freq,
                               .clk_cfg = LEDC_AUTO_CLK};
  ledc_timer_config(&timer);

  ledc_channel_config_t ledc_channel = {.channel = channel,
                                        .duty = 0,
                                        .gpio_num = pwm_pin,
                                        .speed_mode = LEDC_LOW_SPEED_MODE,
                                        .hpoint = 0,
                                        .timer_sel = LEDC_TIMER_0};
  ledc_channel_config(&ledc_channel);
}

void motor_set_speed(uint8_t channel, uint8_t speed_percent) {
  uint32_t duty_max = (1 << LEDC_TIMER_8_BIT) - 1;  // If using 8-bit
  uint32_t duty = (speed_percent * duty_max) / 100;
  ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
}
