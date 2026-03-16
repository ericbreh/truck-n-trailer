#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>

// ESP Pin Definitions
#define MOTOR_IN1_PIN 12
#define MOTOR_IN2_PIN 13
#define ENCODER_A_PIN 33
#define ENCODER_B_PIN 27

// PWM Config
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES LEDC_TIMER_10_BIT
#define LEDC_FREQUENCY 20000

void init_hw(pcnt_unit_handle_t *pcnt_unit) {
  // Setup PWM Timer
  ledc_timer_config_t ledc_timer = {.speed_mode = LEDC_MODE,
                                    .timer_num = LEDC_TIMER,
                                    .duty_resolution = LEDC_DUTY_RES,
                                    .freq_hz = LEDC_FREQUENCY,
                                    .clk_cfg = LEDC_AUTO_CLK};
  ledc_timer_config(&ledc_timer);

  // Setup PWM Channels
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

  // Setup Encoder
  pcnt_unit_config_t unit_config = {
      .high_limit = 32767,
      .low_limit = -32768,
  };
  pcnt_new_unit(&unit_config, pcnt_unit);
  pcnt_glitch_filter_config_t filter_config = {.max_glitch_ns = 1000};
  pcnt_unit_set_glitch_filter(*pcnt_unit, &filter_config);

  // Channel a
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

  // Channel b
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

void app_main(void) {
  pcnt_unit_handle_t pcnt_unit = NULL;
  init_hw(&pcnt_unit);

  int count = 0;
  int prev_count = 0;
  int64_t prev_time = esp_timer_get_time();
  set_motor_speed(300);

  while (1) {
    pcnt_unit_get_count(pcnt_unit, &count);
    int64_t curr_time = esp_timer_get_time();
    int delta = count - prev_count;
    float dt = (curr_time - prev_time) / 1000000.0f;
    float rpm = (delta / 910.0f) * (60.0f / dt);
    printf("dt=%0.2f enc=%d delta=%d rpm=%0.2f\n", dt, count, delta, rpm);

    prev_count = count;
    prev_time = curr_time;
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
