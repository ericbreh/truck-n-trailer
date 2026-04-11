#include "comm_uart.h"
#include "config.h"
#include "hardware.h"
#include "pid.h"

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <inttypes.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

static int target_sign(float value) {
  if (value > CMD_ZERO_RPM_EPS) {
    return 1;
  }
  if (value < -CMD_ZERO_RPM_EPS) {
    return -1;
  }
  return 0;
}

void app_main(void) {
  // Init hardware
  init_pwm();
  comm_uart_init();
  pcnt_unit_handle_t left_encoder = NULL;
  pcnt_unit_handle_t right_encoder = NULL;
  init_encoder(MOTOR_SIDE_LEFT, &left_encoder);
  init_encoder(MOTOR_SIDE_RIGHT, &right_encoder);

  // Timing
  const TickType_t xFrequency = pdMS_TO_TICKS(CONTROL_PERIOD_MS);
  TickType_t xLastWakeTime = xTaskGetTickCount();
  int64_t prev_time = esp_timer_get_time();

  // Encoder
  int delta_l = 0;
  int delta_r = 0;

  // Controller
  float target_rpm_l = DEFAULT_TARGET_RPM;
  float target_rpm_r = DEFAULT_TARGET_RPM;
  PidController drive_pid_l;
  PidController drive_pid_r;
  pid_init(&drive_pid_l, DRIVE_KP, DRIVE_KI, DRIVE_KD);
  pid_init(&drive_pid_r, DRIVE_KP, DRIVE_KI, DRIVE_KD);
  float prev_target_rpm_l = DEFAULT_TARGET_RPM;
  float prev_target_rpm_r = DEFAULT_TARGET_RPM;

  CommandPacket cmd;

  while (1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    // Read command from UART
    if (comm_uart_read_packet(&cmd) == 0) {
      target_rpm_l = cmd.target_rpm_l;
      target_rpm_r = cmd.target_rpm_r;
    } else {
      // Check for stale command timeout
      int64_t age_ms = comm_uart_get_command_age_ms();
      if (age_ms > COMMAND_TIMEOUT_MS) {
        target_rpm_l = DEFAULT_TARGET_RPM;
        target_rpm_r = DEFAULT_TARGET_RPM;
      }
    }

    // Get delta
    pcnt_unit_get_count(left_encoder, &delta_l);
    pcnt_unit_clear_count(left_encoder);
    pcnt_unit_get_count(right_encoder, &delta_r);
    pcnt_unit_clear_count(right_encoder);

    // Get dt
    int64_t curr_time = esp_timer_get_time();
    float dt = (curr_time - prev_time) / 1000000.0f;
    prev_time = curr_time;

    // Get rpm
    float rpm_l =
        (delta_l * LEFT_ENCODER_DIRECTION_SIGN / COUNTS_PER_OUTPUT_REV) *
        (60.0f / dt);
    float rpm_r =
        (delta_r * RIGHT_ENCODER_DIRECTION_SIGN / COUNTS_PER_OUTPUT_REV) *
        (60.0f / dt);

    int sign_l = target_sign(target_rpm_l);
    int sign_r = target_sign(target_rpm_r);
    int prev_sign_l = target_sign(prev_target_rpm_l);
    int prev_sign_r = target_sign(prev_target_rpm_r);

    if (sign_l != 0 && prev_sign_l != 0 && sign_l != prev_sign_l) {
      pid_reset(&drive_pid_l);
    }
    if (sign_r != 0 && prev_sign_r != 0 && sign_r != prev_sign_r) {
      pid_reset(&drive_pid_r);
    }

    int pwm_l = 0;
    int pwm_r = 0;

    if (sign_l == 0) {
      pid_reset(&drive_pid_l);
    } else {
      float error_l = target_rpm_l - rpm_l;
      int pid_pwm_l = pid_compute(&drive_pid_l, error_l, dt, PWM_MAX_DUTY);
      float ff_l = sign_l * FF_STATIC_PWM;
      pwm_l = pid_pwm_l + (int)lroundf(ff_l);
    }

    if (sign_r == 0) {
      pid_reset(&drive_pid_r);
    } else {
      float error_r = target_rpm_r - rpm_r;
      int pid_pwm_r = pid_compute(&drive_pid_r, error_r, dt, PWM_MAX_DUTY);
      float ff_r = sign_r * FF_STATIC_PWM;
      pwm_r = pid_pwm_r + (int)lroundf(ff_r);
    }

    // Send motor control
    set_motor_speed(MOTOR_SIDE_LEFT, pwm_l);
    set_motor_speed(MOTOR_SIDE_RIGHT, pwm_r);

    prev_target_rpm_l = target_rpm_l;
    prev_target_rpm_r = target_rpm_r;

    int64_t cmd_age = comm_uart_get_command_age_ms();
    printf("dt=%3.3f age=%2" PRId64
           " L(cmd=%6.1f rpm=%7.2f pwm=%4d) R(cmd=%6.1f "
           "rpm=%7.2f pwm=%4d)\n",
           dt, cmd_age, target_rpm_l, rpm_l, pwm_l, target_rpm_r, rpm_r, pwm_r);
  }
}
