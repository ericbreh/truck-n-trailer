#pragma once

typedef struct {
  float p;
  float i;
  float d;
  float integral;
  float prev_error;
} PidController;

void pid_init(PidController *pid, float p, float i, float d);
void pid_reset(PidController *pid);
int pid_compute(PidController *pid, float error, float dt, int output_limit);
