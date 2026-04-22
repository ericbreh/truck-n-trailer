#pragma once

#include "driver/gptimer.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

typedef struct {
  gptimer_handle_t control_timer;
  TaskHandle_t task_handle;
} ControlContext;

esp_err_t control_init(ControlContext *ctx);
