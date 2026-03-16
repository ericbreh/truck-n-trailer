#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <stdint.h>
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
#define PWM_MAX_DUTY ((1 << LEDC_DUTY_RES) - 1)

// Encoder Config
#define PCNT_HIGH_LIMIT INT16_MAX
#define PCNT_LOW_LIMIT INT16_MIN
#define PCNT_GLITCH_FILTER_NS 1000
#define COUNTS_PER_OUTPUT_REV 910.0f
#define CONTROL_PERIOD_MS 100

// Control Config
#define STICTION_FF_PWM 380
#define DRIVE_KP 2.5f
#define DRIVE_KI 0.1f
#define DRIVE_KD 0.01f
#define TARGET_RPM 20

static inline int clamp_int(int value, int min_value, int max_value) {
  if (value < min_value) {
    return min_value;
  }
  if (value > max_value) {
    return max_value;
  }
  return value;
}

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
      .high_limit = PCNT_HIGH_LIMIT,
      .low_limit = PCNT_LOW_LIMIT,
  };
  pcnt_new_unit(&unit_config, pcnt_unit);
  pcnt_glitch_filter_config_t filter_config = {.max_glitch_ns =
                                                   PCNT_GLITCH_FILTER_NS};
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

typedef struct {
  float kp;
  float ki;
  float kd;
  float integral;
  float prev_error;
} PidController;

void pid_init(PidController *pid, float p, float i, float d) {
  pid->kp = p;
  pid->ki = i;
  pid->kd = d;
  pid->integral = 0.0f;
  pid->prev_error = 0.0f;
}

int pid_compute(PidController *pid, float error, float dt) {
  float p_out = pid->kp * error;

  pid->integral += error * dt;
  float i_out = pid->ki * pid->integral;

  float derivative = dt > 0.0f ? (error - pid->prev_error) / dt : 0.0f;
  float d_out = pid->kd * derivative;

  pid->prev_error = error;

  float output = p_out + i_out + d_out;
  return (int)roundf(output);
}

void app_main(void) {
  // Init hardware
  pcnt_unit_handle_t pcnt_unit = NULL;
  init_hw(&pcnt_unit);

  // Timing
  const TickType_t xFrequency = pdMS_TO_TICKS(CONTROL_PERIOD_MS);
  TickType_t xLastWakeTime = xTaskGetTickCount();
  int64_t prev_time = esp_timer_get_time();

  // Encoder
  int delta = 0;

  // Controller
  int target_rpm = TARGET_RPM;
  int ff_pwm = target_rpm > 0 ? STICTION_FF_PWM : -STICTION_FF_PWM;
  PidController drive_pid;
  pid_init(&drive_pid, DRIVE_KP, DRIVE_KI, DRIVE_KD);

  while (1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    // Get delta
    pcnt_unit_get_count(pcnt_unit, &delta);
    pcnt_unit_clear_count(pcnt_unit);

    // Get dt
    int64_t curr_time = esp_timer_get_time();
    float dt = (curr_time - prev_time) / 1000000.0f;
    prev_time = curr_time;

    // Get rpm
    float rpm = (delta / COUNTS_PER_OUTPUT_REV) * (60.0f / dt);

    // Calculate output
    float error = target_rpm - rpm;
    int pid_pwm = pid_compute(&drive_pid, error, dt);
    int pwm = pid_pwm + ff_pwm;

    // Send motor control
    set_motor_speed(pwm);

    printf("dt=%0.3f delta=%d rpm=%0.2f error=%0.2f ff=%d pwm=%d\n", dt, delta,
           rpm, error, ff_pwm, pwm);
  }
}
