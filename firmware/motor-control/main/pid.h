#pragma once

typedef struct {
  float kp;
  float ki;
  float kd;
  float integral;
  float prev_error;
} PidController;

void pid_init(PidController *pid, float p, float i, float d);
int pid_compute(PidController *pid, float error, float dt);
