#pragma once

#include <stdbool.h>
#include <stdint.h>

void shared_state_init(void);

void shared_set_target_rpm(float l, float r);
void shared_get_target_rpm(float *l, float *r);

void shared_mark_command_received(void);
int64_t shared_get_command_age_ms(void);

bool shared_kill_is_latched(void);
void shared_latch_kill(void);
