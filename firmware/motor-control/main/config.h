#pragma once

#include "driver/ledc.h"
#include <stdint.h>

// ESP Pin Definitions
#define MOTOR_IN1_PIN 12
#define MOTOR_IN2_PIN 13
#define ENCODER_A_PIN 33
#define ENCODER_B_PIN 27

// PWM Config
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES LEDC_TIMER_10_BIT
#define LEDC_FREQUENCY 20000
#define PWM_MAX_DUTY ((1 << LEDC_DUTY_RES) - 1)

// Encoder Config
#define PCNT_HIGH_LIMIT INT16_MAX
#define PCNT_LOW_LIMIT INT16_MIN
#define PCNT_GLITCH_FILTER_NS 1000
#define COUNTS_PER_OUTPUT_REV 910.0f
#define CONTROL_PERIOD_MS 100

// Control Config
#define STICTION_FF_PWM 400
#define DRIVE_KP 2.5f
#define DRIVE_KI 0.1f
#define DRIVE_KD 0.01f
#define TARGET_RPM 20
