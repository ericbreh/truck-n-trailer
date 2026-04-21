#pragma once

#include "driver/pulse_cnt.h"

typedef enum {
  MOTOR_SIDE_L = 0,
  MOTOR_SIDE_R = 1,
  MOTOR_SIDE_COUNT,
} MotorSide;

void init_pwm(void);
void init_encoder(MotorSide side, pcnt_unit_handle_t *pcnt_unit);
void set_motor_speed(MotorSide side, int duty);
