#include "hardware.h"
#include "config.h"
#include "driver/ledc.h"

static inline int clamp_int(int value, int min_value, int max_value) {
  if (value < min_value) {
    return min_value;
  }
  if (value > max_value) {
    return max_value;
  }
  return value;
}

void init_pwm(void) {
  ledc_timer_config_t ledc_timer = {.speed_mode = LEDC_MODE,
                                    .timer_num = LEDC_TIMER,
                                    .duty_resolution = LEDC_DUTY_RES,
                                    .freq_hz = LEDC_FREQUENCY,
                                    .clk_cfg = LEDC_AUTO_CLK};
  ledc_timer_config(&ledc_timer);

  ledc_channel_config_t chan1 = {.speed_mode = LEDC_MODE,
                                 .channel = LEDC_CHANNEL_0,
                                 .timer_sel = LEDC_TIMER,
                                 .intr_type = LEDC_INTR_DISABLE,
                                 .gpio_num = MOTOR_IN1_PIN,
                                 .duty = 0,
                                 .hpoint = 0};
  ledc_channel_config(&chan1);

  ledc_channel_config_t chan2 = {.speed_mode = LEDC_MODE,
                                 .channel = LEDC_CHANNEL_1,
                                 .timer_sel = LEDC_TIMER,
                                 .intr_type = LEDC_INTR_DISABLE,
                                 .gpio_num = MOTOR_IN2_PIN,
                                 .duty = 0,
                                 .hpoint = 0};
  ledc_channel_config(&chan2);
}

void init_encoder(pcnt_unit_handle_t *pcnt_unit) {
  pcnt_unit_config_t unit_config = {
      .high_limit = PCNT_HIGH_LIMIT,
      .low_limit = PCNT_LOW_LIMIT,
  };
  pcnt_new_unit(&unit_config, pcnt_unit);

  pcnt_glitch_filter_config_t filter_config = {.max_glitch_ns =
                                                   PCNT_GLITCH_FILTER_NS};
  pcnt_unit_set_glitch_filter(*pcnt_unit, &filter_config);

  pcnt_chan_config_t chan_a_config = {
      .edge_gpio_num = ENCODER_A_PIN,
      .level_gpio_num = ENCODER_B_PIN,
  };
  pcnt_channel_handle_t pcnt_chan_a = NULL;
  pcnt_new_channel(*pcnt_unit, &chan_a_config, &pcnt_chan_a);
  pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                               PCNT_CHANNEL_EDGE_ACTION_DECREASE);
  pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

  pcnt_chan_config_t chan_b_config = {
      .edge_gpio_num = ENCODER_B_PIN,
      .level_gpio_num = ENCODER_A_PIN,
  };
  pcnt_channel_handle_t pcnt_chan_b = NULL;
  pcnt_new_channel(*pcnt_unit, &chan_b_config, &pcnt_chan_b);
  pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_DECREASE,
                               PCNT_CHANNEL_EDGE_ACTION_INCREASE);
  pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

  pcnt_unit_enable(*pcnt_unit);
  pcnt_unit_clear_count(*pcnt_unit);
  pcnt_unit_start(*pcnt_unit);
}

void set_motor_speed(int duty) {
  duty = clamp_int(duty, -PWM_MAX_DUTY, PWM_MAX_DUTY);

  if (duty > 0) {
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0, duty);
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_1, 0);
  } else {
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0, 0);
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_1, -duty);
  }
  ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_0);
  ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_1);
}
