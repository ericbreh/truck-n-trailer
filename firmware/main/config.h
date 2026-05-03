#pragma once

#include "driver/ledc.h"
#include <stdint.h>

// ESP Pin Definitions
#define MOTOR_IN1_PIN_R 12
#define MOTOR_IN2_PIN_R 13
#define ENCODER_A_PIN_R 27
#define ENCODER_B_PIN_R 33

#define MOTOR_IN1_PIN_L 14
#define MOTOR_IN2_PIN_L 15
#define ENCODER_A_PIN_L 32
#define ENCODER_B_PIN_L 20

// Direction Polarity
#define MOTOR_DIRECTION_SIGN_R -1
#define MOTOR_DIRECTION_SIGN_L -1
#define ENCODER_DIRECTION_SIGN_R -1
#define ENCODER_DIRECTION_SIGN_L 1

// PWM
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES LEDC_TIMER_10_BIT
#define LEDC_FREQUENCY 20000
#define PWM_MAX_DUTY ((1 << LEDC_DUTY_RES) - 1)

// Encoder
#define COUNTS_PER_OUTPUT_REV 910.0f
#define CONTROL_PERIOD_MS 10

// Task
#define CONTROL_TASK_STACK_WORDS 4096
#define COMM_TASK_STACK_WORDS 4096
#define CONTROL_TASK_PRIORITY 10
#define COMM_TASK_PRIORITY 8

// Control
#define FF_STATIC_PWM 400
#define DRIVE_KP 10.0f
#define DRIVE_KI 10.0f
#define DRIVE_KD 0.0f

// Runtime Command
#define DEFAULT_TARGET_RPM 0.0f
#define COMMAND_TIMEOUT_MS 100
#define CMD_ZERO_RPM_EPS 0.01f

// UART Command Interface
#define UART_BAUD_RATE 115200
#define UART_TX_PIN 1
#define UART_RX_PIN 3
#define UART_RX_BUFFER_SIZE 256
#define UART_TX_BUFFER_SIZE 256
#define UART_COMMAND_MAX_LEN 64
#define UART_EVENT_QUEUE_LEN 16

// Kill Switch
#define KILL_SWITCH_GPIO_PIN 38

// Potentiometer
#define POT_TRAVEL_DEG 270.0f
#define POT_RAW_MIN 0
#define POT_RAW_MAX 4095
#define POT_ZERO_OFFSET_DEG 0.0f
