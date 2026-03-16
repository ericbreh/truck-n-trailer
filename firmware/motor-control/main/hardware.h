#pragma once

#include "driver/pulse_cnt.h"

void init_pwm(void);
void init_encoder(pcnt_unit_handle_t *pcnt_unit);
void set_motor_speed(int duty);
