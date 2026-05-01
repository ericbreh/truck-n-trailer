#include "control.h"

#include "config.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "hardware.h"
#include "killswitch.h"
#include "pid.h"
#include "shared_state.h"

#include <math.h>
#include <stdio.h>

static const char *TAG = "control";
static adc_oneshot_unit_handle_t s_pot_adc_handle = NULL;
static bool s_pot_adc_ready = false;

#define POT_ADC_UNIT ADC_UNIT_1
#define POT_ADC_CHANNEL ADC_CHANNEL_6

static int target_sign(float value) {
  if (value > CMD_ZERO_RPM_EPS) {
    return 1;
  }
  if (value < -CMD_ZERO_RPM_EPS) {
    return -1;
  }
  return 0;
}

static bool pot_adc_init(void) {
  adc_oneshot_unit_init_cfg_t unit_cfg = {
      .unit_id = POT_ADC_UNIT,
      .ulp_mode = ADC_ULP_MODE_DISABLE,
  };
  esp_err_t err = adc_oneshot_new_unit(&unit_cfg, &s_pot_adc_handle);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "pot adc unit init failed (%s)", esp_err_to_name(err));
    return false;
  }

  adc_oneshot_chan_cfg_t chan_cfg = {
      .atten = ADC_ATTEN_DB_12,
      .bitwidth = ADC_BITWIDTH_DEFAULT,
  };
  err = adc_oneshot_config_channel(s_pot_adc_handle, POT_ADC_CHANNEL, &chan_cfg);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "pot adc channel config failed (%s)", esp_err_to_name(err));
    adc_oneshot_del_unit(s_pot_adc_handle);
    s_pot_adc_handle = NULL;
    return false;
  }

  return true;
}

static float read_pot_angle_deg(void) {
  if (!s_pot_adc_ready || s_pot_adc_handle == NULL) {
    return 0.0f;
  }

  int raw = 0;
  esp_err_t err = adc_oneshot_read(s_pot_adc_handle, POT_ADC_CHANNEL, &raw);
  if (err != ESP_OK) {
    return 0.0f;
  }

  int raw_min = POT_RAW_MIN;
  int raw_max = POT_RAW_MAX;
  if (raw_max <= raw_min) {
    return 0.0f;
  }
  if (raw < raw_min) {
    raw = raw_min;
  } else if (raw > raw_max) {
    raw = raw_max;
  }

  // Map calibrated raw range to the physical pot travel, then center at 0.
  const float norm = (float)(raw - raw_min) / (float)(raw_max - raw_min);
  const float angle_centered = norm * POT_TRAVEL_DEG - (POT_TRAVEL_DEG * 0.5f);
  return angle_centered + POT_ZERO_OFFSET_DEG;
}

static bool IRAM_ATTR on_control_timer_alarm(
    gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata,
    void *user_ctx) {
  (void)timer;
  (void)edata;
  TaskHandle_t control_task_handle = (TaskHandle_t)user_ctx;
  BaseType_t higher_priority_task_woken = pdFALSE;
  vTaskNotifyGiveFromISR(control_task_handle, &higher_priority_task_woken);
  return higher_priority_task_woken == pdTRUE;
}

static void control_task_entry(void *arg) {
  (void)arg;
  const float dt = (float)CONTROL_PERIOD_MS / 1000.0f;

  PidController drive_pid_l;
  PidController drive_pid_r;
  pid_init(&drive_pid_l, DRIVE_KP, DRIVE_KI, DRIVE_KD);
  pid_init(&drive_pid_r, DRIVE_KP, DRIVE_KI, DRIVE_KD);

  float prev_target_rpm_l = DEFAULT_TARGET_RPM;
  float prev_target_rpm_r = DEFAULT_TARGET_RPM;
  bool in_kill_state = false;
  int telemetry_div = 0;

  while (1) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // Handle kill switch
    if (shared_kill_is_latched()) {
      if (!in_kill_state) {
        pid_reset(&drive_pid_l);
        pid_reset(&drive_pid_r);
        in_kill_state = true;
      }
      set_motor_speed(MOTOR_SIDE_L, 0);
      set_motor_speed(MOTOR_SIDE_R, 0);
      continue;
    }

    in_kill_state = false;

    // Get target RPM
    float target_rpm_l = DEFAULT_TARGET_RPM;
    float target_rpm_r = DEFAULT_TARGET_RPM;
    shared_get_target_rpm(&target_rpm_l, &target_rpm_r);
    if (shared_get_command_age_ms() > COMMAND_TIMEOUT_MS) {
      target_rpm_l = DEFAULT_TARGET_RPM;
      target_rpm_r = DEFAULT_TARGET_RPM;
    }

    // Sample encoders and convert counts to RPM
    int delta_l = (int)encoder_consume_delta(MOTOR_SIDE_L);
    int delta_r = (int)encoder_consume_delta(MOTOR_SIDE_R);

    float rpm_l = (delta_l * ENCODER_DIRECTION_SIGN_L / COUNTS_PER_OUTPUT_REV) *
                  (60.0f / dt);
    float rpm_r = (delta_r * ENCODER_DIRECTION_SIGN_R / COUNTS_PER_OUTPUT_REV) *
                  (60.0f / dt);

    int sign_l = target_sign(target_rpm_l);
    int sign_r = target_sign(target_rpm_r);
    int prev_sign_l = target_sign(prev_target_rpm_l);
    int prev_sign_r = target_sign(prev_target_rpm_r);

    // Reset PID when crossing through zero
    if (sign_l != 0 && prev_sign_l != 0 && sign_l != prev_sign_l) {
      pid_reset(&drive_pid_l);
    }
    if (sign_r != 0 && prev_sign_r != 0 && sign_r != prev_sign_r) {
      pid_reset(&drive_pid_r);
    }

    int pwm_l = 0;
    int pwm_r = 0;

    // PID + feedforward per side
    if (sign_l == 0) {
      pid_reset(&drive_pid_l);
    } else {
      float error_l = target_rpm_l - rpm_l;
      int pid_pwm_l = pid_compute(&drive_pid_l, error_l, dt, PWM_MAX_DUTY);
      float ff_l = sign_l * FF_STATIC_PWM;
      pwm_l = pid_pwm_l + (int)lroundf(ff_l);
    }

    // Reset PID when target is zero
    if (sign_r == 0) {
      pid_reset(&drive_pid_r);
    } else {
      float error_r = target_rpm_r - rpm_r;
      int pid_pwm_r = pid_compute(&drive_pid_r, error_r, dt, PWM_MAX_DUTY);
      float ff_r = sign_r * FF_STATIC_PWM;
      pwm_r = pid_pwm_r + (int)lroundf(ff_r);
    }

    // Apply motor outputs
    set_motor_speed(MOTOR_SIDE_L, pwm_l);
    set_motor_speed(MOTOR_SIDE_R, pwm_r);

    prev_target_rpm_l = target_rpm_l;
    prev_target_rpm_r = target_rpm_r;

    printf("dt=%3.3f age=%lld L(cmd=%6.1f rpm=%7.2f pwm=%4d) R(cmd=%6.1f "
           "rpm=%7.2f pwm=%4d) kill=%d\n",
           dt, (long long)shared_get_command_age_ms(), target_rpm_l, rpm_l,
           pwm_l, target_rpm_r, rpm_r, pwm_r, shared_kill_is_latched() ? 1 : 0);

    telemetry_div++;
    if (telemetry_div >= 10) {
      telemetry_div = 0;
      printf("POT,%.2f\n", read_pot_angle_deg());
    }
  }
}

esp_err_t control_init(ControlContext *ctx) {
  s_pot_adc_ready = pot_adc_init();
  if (!s_pot_adc_ready) {
    ESP_LOGW(TAG, "pot telemetry disabled");
  }

  // Start control task
  if (xTaskCreate(control_task_entry, "control_task", CONTROL_TASK_STACK_WORDS,
                  ctx, CONTROL_TASK_PRIORITY, &ctx->task_handle) != pdPASS) {
    ESP_LOGE(TAG, "failed to create control task");
    return ESP_FAIL;
  }

  // Init killswitch
  esp_err_t err = killswitch_init(ctx->task_handle);
  if (err != ESP_OK) {
    return err;
  }

  // Start timer
  gptimer_config_t timer_cfg = {
      .clk_src = GPTIMER_CLK_SRC_DEFAULT,
      .direction = GPTIMER_COUNT_UP,
      .resolution_hz = 1000000,
  };
  err = gptimer_new_timer(&timer_cfg, &ctx->control_timer);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "failed to create control timer (%s)", esp_err_to_name(err));
    return err;
  }

  // Register timer alarm callback
  gptimer_event_callbacks_t callbacks = {
      .on_alarm = on_control_timer_alarm,
  };
  err = gptimer_register_event_callbacks(ctx->control_timer, &callbacks,
                                         ctx->task_handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "failed to register timer callback (%s)",
             esp_err_to_name(err));
    return err;
  }

  // Set periodic alarm
  gptimer_alarm_config_t alarm_cfg = {
      .alarm_count = CONTROL_PERIOD_MS * 1000ULL,
      .reload_count = 0,
      .flags = {.auto_reload_on_alarm = true},
  };
  err = gptimer_set_alarm_action(ctx->control_timer, &alarm_cfg);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "failed to set timer alarm (%s)", esp_err_to_name(err));
    return err;
  }

  // Enable and start timer
  err = gptimer_enable(ctx->control_timer);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "failed to enable timer (%s)", esp_err_to_name(err));
    return err;
  }

  err = gptimer_start(ctx->control_timer);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "failed to start timer (%s)", esp_err_to_name(err));
    return err;
  }

  return ESP_OK;
}
