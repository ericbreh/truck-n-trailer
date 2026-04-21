#include "communication.h"
#include "control.h"
#include "esp_log.h"
#include "hardware.h"
#include "shared_state.h"

static const char *TAG = "main";

static ControlContext control_ctx = {
    .left_encoder = NULL,
    .right_encoder = NULL,
    .control_timer = NULL,
    .task_handle = NULL,
};

void app_main(void) {
  shared_state_init();
  init_pwm();

  esp_err_t err = control_init(&control_ctx);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "control init failed (%s)", esp_err_to_name(err));
    return;
  }

  err = communication_init();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "communication init failed (%s)", esp_err_to_name(err));
    return;
  }

  ESP_LOGI(TAG, "System initialized with task-driven control architecture");
}
