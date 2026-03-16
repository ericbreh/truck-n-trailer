#include "config.h"
#include "hardware.h"
#include "pid.h"

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdint.h>
#include <stdio.h>

void app_main(void) {
  // Init hardware
  init_pwm();
  pcnt_unit_handle_t pcnt_unit = NULL;
  init_encoder(&pcnt_unit);

  // Timing
  const TickType_t xFrequency = pdMS_TO_TICKS(CONTROL_PERIOD_MS);
  TickType_t xLastWakeTime = xTaskGetTickCount();
  int64_t prev_time = esp_timer_get_time();

  // Encoder
  int delta = 0;

  // Controller
  const int target_rpm = TARGET_RPM;
  const int ff_pwm = target_rpm > 0 ? STICTION_FF_PWM : -STICTION_FF_PWM;
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
