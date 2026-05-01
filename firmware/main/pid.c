#include "pid.h"
#include <math.h>

void pid_init(PidController *pid, float p, float i, float d) {
  pid->p = p;
  pid->i = i;
  pid->d = d;
  pid->integral = 0.0f;
  pid->prev_error = 0.0f;
}

void pid_reset(PidController *pid) {
  pid->integral = 0.0f;
  pid->prev_error = 0.0f;
}

int pid_compute(PidController *pid, float error, float dt, int output_limit) {
  float p_out = pid->p * error;

  float derivative = dt > 0.0f ? (error - pid->prev_error) / dt : 0.0f;
  float d_out = pid->d * derivative;

  float output_without_i = p_out + d_out;
  if (output_without_i >= -output_limit && output_without_i <= output_limit) {
    pid->integral += error * dt;
  }
  float i_out = pid->i * pid->integral;

  pid->prev_error = error;

  float output = p_out + i_out + d_out;
  output = fmaxf(-output_limit, fminf(output_limit, output));
  return (int)roundf(output);
}
