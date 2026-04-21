#pragma once

#include "driver/gptimer.h"
#include "driver/pulse_cnt.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

typedef struct {
  pcnt_unit_handle_t left_encoder;
  pcnt_unit_handle_t right_encoder;
  gptimer_handle_t control_timer;
  TaskHandle_t task_handle;
} ControlContext;

esp_err_t control_create(ControlContext *ctx);
esp_err_t control_start_timer(ControlContext *ctx);
esp_err_t control_init(ControlContext *ctx);
