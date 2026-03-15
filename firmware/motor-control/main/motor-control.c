#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
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
  pcnt_unit_config_t unit_config = {.high_limit = 20000, .low_limit = -20000};
  pcnt_new_unit(&unit_config, pcnt_unit);

  pcnt_chan_config_t chan_a_config = {.edge_gpio_num = ENCODER_A_PIN,
                                      .level_gpio_num = ENCODER_B_PIN};
  pcnt_channel_handle_t pcnt_chan = NULL;
  pcnt_new_channel(*pcnt_unit, &chan_a_config, &pcnt_chan);

  pcnt_channel_set_edge_action(pcnt_chan, PCNT_CHANNEL_EDGE_ACTION_DECREASE,
                               PCNT_CHANNEL_EDGE_ACTION_INCREASE);
  pcnt_channel_set_level_action(pcnt_chan, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
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
  while (1) {
    // printf("Moving Forward...\n");
    // set_motor_speed(500);
    // vTaskDelay(pdMS_TO_TICKS(2000));

    pcnt_unit_get_count(pcnt_unit, &count);
    printf("Stop. Encoder: %d\n", count);
    set_motor_speed(0);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
