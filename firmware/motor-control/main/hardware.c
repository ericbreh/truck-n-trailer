#include "hardware.h"
#include "config.h"
#include "driver/ledc.h"
#include <stddef.h>

typedef struct {
  int in1_pin;
  int in2_pin;
  ledc_channel_t in1_channel;
  ledc_channel_t in2_channel;
  int direction_sign;
} MotorPwmConfig;

typedef struct {
  int encoder_a_pin;
  int encoder_b_pin;
} EncoderPinConfig;

static const MotorPwmConfig motor_pwm_cfg[MOTOR_SIDE_COUNT] = {
    [MOTOR_SIDE_L] =
        {
            .in1_pin = MOTOR_IN1_PIN_L,
            .in2_pin = MOTOR_IN2_PIN_L,
            .in1_channel = LEDC_CHANNEL_0,
            .in2_channel = LEDC_CHANNEL_1,
            .direction_sign = MOTOR_DIRECTION_SIGN_L,
        },
    [MOTOR_SIDE_R] =
        {
            .in1_pin = MOTOR_IN1_PIN_R,
            .in2_pin = MOTOR_IN2_PIN_R,
            .in1_channel = LEDC_CHANNEL_2,
            .in2_channel = LEDC_CHANNEL_3,
            .direction_sign = MOTOR_DIRECTION_SIGN_R,
        },
};

static const EncoderPinConfig encoder_pin_cfg[MOTOR_SIDE_COUNT] = {
    [MOTOR_SIDE_L] =
        {
            .encoder_a_pin = ENCODER_A_PIN_L,
            .encoder_b_pin = ENCODER_B_PIN_L,
        },
    [MOTOR_SIDE_R] =
        {
            .encoder_a_pin = ENCODER_A_PIN_R,
            .encoder_b_pin = ENCODER_B_PIN_R,
        },
};

void init_pwm(void) {
  // Configure LEDC timer for motor PWM frequency
  ledc_timer_config_t ledc_timer = {.speed_mode = LEDC_MODE,
                                    .timer_num = LEDC_TIMER,
                                    .duty_resolution = LEDC_DUTY_RES,
                                    .freq_hz = LEDC_FREQUENCY,
                                    .clk_cfg = LEDC_AUTO_CLK};
  ledc_timer_config(&ledc_timer);

  // Configure IN1/IN2 channels per motor
  for (int side = 0; side < MOTOR_SIDE_COUNT; side++) {
    ledc_channel_config_t in1_cfg = {
        .speed_mode = LEDC_MODE,
        .channel = motor_pwm_cfg[side].in1_channel,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = motor_pwm_cfg[side].in1_pin,
        .duty = 0,
        .hpoint = 0,
    };
    ledc_channel_config(&in1_cfg);

    ledc_channel_config_t in2_cfg = {
        .speed_mode = LEDC_MODE,
        .channel = motor_pwm_cfg[side].in2_channel,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = motor_pwm_cfg[side].in2_pin,
        .duty = 0,
        .hpoint = 0,
    };
    ledc_channel_config(&in2_cfg);
  }
}

void init_encoder(MotorSide side, pcnt_unit_handle_t *pcnt_unit) {
  const EncoderPinConfig encoder_cfg = encoder_pin_cfg[side];

  // Create PCNT unit and glitch filter
  pcnt_unit_config_t unit_config = {
      .high_limit = PCNT_HIGH_LIMIT,
      .low_limit = PCNT_LOW_LIMIT,
  };
  pcnt_new_unit(&unit_config, pcnt_unit);

  pcnt_glitch_filter_config_t filter_config = {.max_glitch_ns =
                                                   PCNT_GLITCH_FILTER_NS};
  pcnt_unit_set_glitch_filter(*pcnt_unit, &filter_config);

  // Quadrature channel A (edge on A, level from B)
  pcnt_chan_config_t chan_a_config = {
      .edge_gpio_num = encoder_cfg.encoder_a_pin,
      .level_gpio_num = encoder_cfg.encoder_b_pin,
  };
  pcnt_channel_handle_t pcnt_chan_a = NULL;
  pcnt_new_channel(*pcnt_unit, &chan_a_config, &pcnt_chan_a);
  pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                               PCNT_CHANNEL_EDGE_ACTION_DECREASE);
  pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

  // Quadrature channel B (edge on B, level from A)
  pcnt_chan_config_t chan_b_config = {
      .edge_gpio_num = encoder_cfg.encoder_b_pin,
      .level_gpio_num = encoder_cfg.encoder_a_pin,
  };
  pcnt_channel_handle_t pcnt_chan_b = NULL;
  pcnt_new_channel(*pcnt_unit, &chan_b_config, &pcnt_chan_b);
  pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_DECREASE,
                               PCNT_CHANNEL_EDGE_ACTION_INCREASE);
  pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

  // Enable counter and start sampling
  pcnt_unit_enable(*pcnt_unit);
  pcnt_unit_clear_count(*pcnt_unit);
  pcnt_unit_start(*pcnt_unit);
}

void set_motor_speed(MotorSide side, int duty) {
  const MotorPwmConfig motor_cfg = motor_pwm_cfg[side];

  // Apply direction sign and clamp duty
  duty *= motor_cfg.direction_sign;

  if (duty > PWM_MAX_DUTY) {
    duty = PWM_MAX_DUTY;
  } else if (duty < -PWM_MAX_DUTY) {
    duty = -PWM_MAX_DUTY;
  }

  // PWM on IN1 for forward, IN2 for reverse
  if (duty > 0) {
    ledc_set_duty(LEDC_MODE, motor_cfg.in1_channel, duty);
    ledc_set_duty(LEDC_MODE, motor_cfg.in2_channel, 0);
  } else {
    ledc_set_duty(LEDC_MODE, motor_cfg.in1_channel, 0);
    ledc_set_duty(LEDC_MODE, motor_cfg.in2_channel, -duty);
  }
  ledc_update_duty(LEDC_MODE, motor_cfg.in1_channel);
  ledc_update_duty(LEDC_MODE, motor_cfg.in2_channel);
}
