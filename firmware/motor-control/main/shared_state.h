#pragma once

#include "freertos/FreeRTOS.h"
#include <stdbool.h>
#include <stdint.h>

void shared_state_init(void);

void shared_set_target_rpm(float left, float right);
void shared_get_target_rpm(float *left, float *right);

void shared_mark_command_received(void);
int64_t shared_get_command_age_ms(void);

bool shared_kill_is_latched(void);
void shared_latch_kill_from_isr(BaseType_t *hp_woken);
