#include "shared_state.h"

#include "esp_timer.h"
#include "freertos/portmacro.h"
#include <limits.h>

static portMUX_TYPE state_lock = portMUX_INITIALIZER_UNLOCKED;

static float target_rpm_left = 0.0f;
static float target_rpm_right = 0.0f;
static int64_t last_command_rx_ms = -1;
static bool kill_latched = false;

void shared_state_init(void) {
  taskENTER_CRITICAL(&state_lock);
  target_rpm_left = 0.0f;
  target_rpm_right = 0.0f;
  last_command_rx_ms = -1;
  kill_latched = false;
  taskEXIT_CRITICAL(&state_lock);
}

void shared_set_target_rpm(float left, float right) {
  taskENTER_CRITICAL(&state_lock);
  target_rpm_left = left;
  target_rpm_right = right;
  taskEXIT_CRITICAL(&state_lock);
}

void shared_get_target_rpm(float *left, float *right) {
  taskENTER_CRITICAL(&state_lock);
  *left = target_rpm_left;
  *right = target_rpm_right;
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

void shared_latch_kill_from_isr(BaseType_t *hp_woken) {
  (void)hp_woken;
  taskENTER_CRITICAL_ISR(&state_lock);
  kill_latched = true;
  taskEXIT_CRITICAL_ISR(&state_lock);
}
