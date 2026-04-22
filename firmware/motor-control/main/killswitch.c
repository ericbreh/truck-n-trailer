#include "killswitch.h"

#include "config.h"
#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "shared_state.h"

static const char *TAG = "killswitch";

static void IRAM_ATTR on_kill_switch_isr(void *arg) {
  TaskHandle_t control_task_handle = (TaskHandle_t)arg;
  BaseType_t higher_priority_task_woken = pdFALSE;
  shared_latch_kill();
  vTaskNotifyGiveFromISR(control_task_handle, &higher_priority_task_woken);
  if (higher_priority_task_woken == pdTRUE) {
    portYIELD_FROM_ISR();
  }
}

esp_err_t killswitch_init(TaskHandle_t control_task_handle) {
  // Configure kill GPIO
  gpio_config_t io_cfg = {
      .pin_bit_mask = (1ULL << KILL_SWITCH_GPIO_PIN),
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_ENABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_NEGEDGE,
  };

  esp_err_t err = gpio_config(&io_cfg);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "failed to configure kill gpio (%s)", esp_err_to_name(err));
    return err;
  }

  // Install GPIO ISR service
  err = gpio_install_isr_service(0);
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
    ESP_LOGE(TAG, "failed to install gpio isr service (%s)",
             esp_err_to_name(err));
    return err;
  }

  // Attach kill interrupt handler
  err = gpio_isr_handler_add(KILL_SWITCH_GPIO_PIN, on_kill_switch_isr,
                             control_task_handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "failed to add kill gpio isr (%s)", esp_err_to_name(err));
    return err;
  }

  return ESP_OK;
}
