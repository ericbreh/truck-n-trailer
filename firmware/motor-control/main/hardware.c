#include "hardware.h"
#include "config.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_attr.h"
#include "esp_err.h"
#include <stdatomic.h>
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

typedef struct {
  int pin_a;
  int pin_b;
  uint8_t last_state;
  _Atomic int32_t pending;
} EncoderState;

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

static EncoderState s_enc[MOTOR_SIDE_COUNT];

// ┌───────────┬───────────┬───────────┬───────────┬───────┬──────┐
// │ old (A,B) │ old_state │ new (A,B) │ new_state │ index │ step │
// ├───────────┼───────────┼───────────┼───────────┼───────┼──────┤
// │ 00        │ 0         │ 00        │ 0         │ 0     │ 0    │
// │ 00        │ 0         │ 01        │ 1         │ 1     │ +1   │
// │ 00        │ 0         │ 10        │ 2         │ 2     │ -1   │
// │ 00        │ 0         │ 11        │ 3         │ 3     │ 0    │
// │ 01        │ 1         │ 00        │ 0         │ 4     │ -1   │
// │ 01        │ 1         │ 01        │ 1         │ 5     │ 0    │
// │ 01        │ 1         │ 10        │ 2         │ 6     │ 0    │
// │ 01        │ 1         │ 11        │ 3         │ 7     │ +1   │
// │ 10        │ 2         │ 00        │ 0         │ 8     │ +1   │
// │ 10        │ 2         │ 01        │ 1         │ 9     │ 0    │
// │ 10        │ 2         │ 10        │ 2         │ 10    │ 0    │
// │ 10        │ 2         │ 11        │ 3         │ 11    │ -1   │
// │ 11        │ 3         │ 00        │ 0         │ 12    │ 0    │
// │ 11        │ 3         │ 01        │ 1         │ 13    │ -1   │
// │ 11        │ 3         │ 10        │ 2         │ 14    │ +1   │
// │ 11        │ 3         │ 11        │ 3         │ 15    │ 0    │
// └───────────┴───────────┴───────────┴───────────┴───────┴──────┘
static const int8_t transition_table[16] = {
    0, +1, -1, 0, -1, 0, 0, +1, +1, 0, 0, -1, 0, -1, +1, 0,
};

static void IRAM_ATTR encoder_gpio_isr(void *arg) {
  EncoderState *e = (EncoderState *)arg;

  const int a = gpio_get_level(e->pin_a);
  const int b = gpio_get_level(e->pin_b);

  // State = (A << 1) | B
  const uint8_t new_state = (uint8_t)(((a & 1) << 1) | (b & 1));

  // Index = (last_state << 2) | new_state
  const uint8_t index = (uint8_t)((e->last_state << 2) | new_state);

  const int8_t step = transition_table[index];
  e->last_state = new_state;

  if (step != 0) {
    atomic_fetch_add(&e->pending, step);
  }
}

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

static void init_encoder_side(MotorSide side) {
  const EncoderPinConfig *enc = &encoder_pin_cfg[side];

  // Install GPIO ISR service
  esp_err_t err = gpio_install_isr_service(0);
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
    ESP_ERROR_CHECK(err);
  }

  gpio_reset_pin(enc->encoder_a_pin);
  gpio_reset_pin(enc->encoder_b_pin);

  // Configure GPIO
  gpio_config_t io_cfg = {
      .pin_bit_mask =
          (1ULL << enc->encoder_a_pin) | (1ULL << enc->encoder_b_pin),
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  ESP_ERROR_CHECK(gpio_config(&io_cfg));

  // Seed runtime state
  EncoderState *st = &s_enc[side];
  st->pin_a = enc->encoder_a_pin;
  st->pin_b = enc->encoder_b_pin;
  const int a = gpio_get_level(st->pin_a);
  const int b = gpio_get_level(st->pin_b);
  st->last_state = (uint8_t)(((a & 1) << 1) | (b & 1));
  atomic_store(&st->pending, 0);

  // Trigger on both rising and falling edges.
  ESP_ERROR_CHECK(gpio_set_intr_type(st->pin_a, GPIO_INTR_ANYEDGE));
  ESP_ERROR_CHECK(gpio_set_intr_type(st->pin_b, GPIO_INTR_ANYEDGE));
  ESP_ERROR_CHECK(gpio_isr_handler_add(st->pin_a, encoder_gpio_isr, st));
  ESP_ERROR_CHECK(gpio_isr_handler_add(st->pin_b, encoder_gpio_isr, st));
}

void init_encoders(void) {
  for (int side = 0; side < MOTOR_SIDE_COUNT; side++) {
    init_encoder_side((MotorSide)side);
  }
}

int32_t encoder_consume_delta(MotorSide side) {
  return atomic_exchange(&s_enc[side].pending, 0);
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
