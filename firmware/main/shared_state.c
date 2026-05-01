#include "shared_state.h"

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

static portMUX_TYPE state_lock = portMUX_INITIALIZER_UNLOCKED;

static float target_rpm_l = 0.0f;
static float target_rpm_r = 0.0f;
static int64_t last_command_rx_ms = -1;
static bool kill_latched = false;

void shared_state_init(void) {
  taskENTER_CRITICAL(&state_lock);
  target_rpm_l = 0.0f;
  target_rpm_r = 0.0f;
  last_command_rx_ms = -1;
  kill_latched = false;
  taskEXIT_CRITICAL(&state_lock);
}

void shared_set_target_rpm(float l, float r) {
  taskENTER_CRITICAL(&state_lock);
  target_rpm_l = l;
  target_rpm_r = r;
  taskEXIT_CRITICAL(&state_lock);
}

void shared_get_target_rpm(float *l, float *r) {
  taskENTER_CRITICAL(&state_lock);
  *l = target_rpm_l;
  *r = target_rpm_r;
  taskEXIT_CRITICAL(&state_lock);
}

void shared_mark_command_received(void) {
  taskENTER_CRITICAL(&state_lock);
  last_command_rx_ms = esp_timer_get_time() / 1000;
  taskEXIT_CRITICAL(&state_lock);
}

int64_t shared_get_command_age_ms(void) {
  int64_t last_ms = -1;
  taskENTER_CRITICAL(&state_lock);
  last_ms = last_command_rx_ms;
  taskEXIT_CRITICAL(&state_lock);

  if (last_ms < 0) {
    return INT64_MAX;
  }

  return (esp_timer_get_time() / 1000) - last_ms;
}

bool shared_kill_is_latched(void) {
  bool latched = false;
  taskENTER_CRITICAL(&state_lock);
  latched = kill_latched;
  taskEXIT_CRITICAL(&state_lock);
  return latched;
}

void shared_latch_kill(void) {
  taskENTER_CRITICAL_ISR(&state_lock);
  kill_latched = true;
  taskEXIT_CRITICAL_ISR(&state_lock);
}
