#include "pid.h"
#include <math.h>

void pid_init(PidController *pid, float p, float i, float d) {
  pid->kp = p;
  pid->ki = i;
  pid->kd = d;
  pid->integral = 0.0f;
  pid->prev_error = 0.0f;
}

int pid_compute(PidController *pid, float error, float dt, int output_limit) {
  float p_out = pid->kp * error;

  float derivative = dt > 0.0f ? (error - pid->prev_error) / dt : 0.0f;
  float d_out = pid->kd * derivative;

  float output_without_i = p_out + d_out;
  if (output_without_i >= -output_limit && output_without_i <= output_limit) {
    pid->integral += error * dt;
  }
  float i_out = pid->ki * pid->integral;

  pid->prev_error = error;

  float output = p_out + i_out + d_out;
  output = fmaxf(-output_limit, fminf(output_limit, output));
  return (int)roundf(output);
}
