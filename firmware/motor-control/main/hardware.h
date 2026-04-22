#pragma once

#include <stdint.h>

typedef enum {
  MOTOR_SIDE_L = 0,
  MOTOR_SIDE_R = 1,
  MOTOR_SIDE_COUNT,
} MotorSide;

void init_pwm(void);
void init_encoders(void);
int32_t encoder_consume_delta(MotorSide side);
void set_motor_speed(MotorSide side, int duty);
