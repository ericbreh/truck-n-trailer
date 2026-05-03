#include "communication.h"

#include "config.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "shared_state.h"

#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define UART_NUM UART_NUM_0
#define CMD_START "C,"

static const char *TAG = "communication";
static QueueHandle_t uart_event_queue = NULL;

typedef struct {
  float target_rpm_l;
  float target_rpm_r;
} CommandPacket;

uint16_t crc16_modbus(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++) {
      if (crc & 0x0001) {
        crc = (crc >> 1) ^ 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

static bool parse_float_token(const char *token, float *out) {
  char *end = NULL;
  errno = 0;
  float value = strtof(token, &end);
  if (errno != 0 || end == token || *end != '\0' || !isfinite(value)) {
    return false;
  }
  *out = value;
  return true;
}

static bool parse_crc_token(const char *token, uint16_t *out) {
  char *end = NULL;
  errno = 0;
  unsigned long value = strtoul(token, &end, 16);
  if (errno != 0 || end == token || *end != '\0' || value > UINT16_MAX) {
    return false;
  }
  *out = (uint16_t)value;
  return true;
}

static int parse_command_line(const char *line, CommandPacket *pkt) {
  if (strncmp(line, CMD_START, sizeof(CMD_START) - 1) != 0) {
    return -1;
  }

  const char *crc_sep = strrchr(line, ',');
  if (crc_sep == NULL || crc_sep <= line + (sizeof(CMD_START) - 1)) {
    return -1;
  }

  uint16_t received_crc = 0;
  if (!parse_crc_token(crc_sep + 1, &received_crc)) {
    return -1;
  }

  uint16_t computed_crc =
      crc16_modbus((const uint8_t *)line, (size_t)(crc_sep - line));
  if (computed_crc != received_crc) {
    return -1;
  }

  char payload[UART_COMMAND_MAX_LEN];
  size_t payload_len = (size_t)(crc_sep - line);
  if (payload_len >= sizeof(payload)) {
    return -1;
  }
  memcpy(payload, line, payload_len);
  payload[payload_len] = '\0';

  char *body = payload + (sizeof(CMD_START) - 1);
  char *saveptr = NULL;
  char *rpm_l_token = strtok_r(body, ",", &saveptr);
  char *rpm_r_token = strtok_r(NULL, ",", &saveptr);
  char *extra_token = strtok_r(NULL, ",", &saveptr);

  if (rpm_l_token == NULL || rpm_r_token == NULL || extra_token != NULL) {
    return -1;
  }

  float rpm_l = 0.0f;
  float rpm_r = 0.0f;
  if (!parse_float_token(rpm_l_token, &rpm_l) ||
      !parse_float_token(rpm_r_token, &rpm_r)) {
    return -1;
  }

  pkt->target_rpm_l = rpm_l;
  pkt->target_rpm_r = rpm_r;
  return 0;
}

static void process_pattern_event(void) {
  // Get newline-terminated line length from pattern queue
  int pattern_pos = uart_pattern_pop_pos(UART_NUM);
  if (pattern_pos < 0) {
    return;
  }

  // Discard oversize lines so UART RX stays in sync
  size_t line_len = (size_t)pattern_pos + 1;
  if (line_len >= UART_COMMAND_MAX_LEN) {
    uint8_t discard[UART_COMMAND_MAX_LEN];
    uart_read_bytes(UART_NUM, discard, line_len, pdMS_TO_TICKS(0));
    return;
  }

  // Read full line into buffer
  char line[UART_COMMAND_MAX_LEN];
  int line_read = uart_read_bytes(UART_NUM, line, line_len, pdMS_TO_TICKS(0));
  if (line_read <= 0) {
    return;
  }

  // Strip trailing CR/LF
  size_t logical_len = (size_t)line_read;
  while (logical_len > 0 &&
         (line[logical_len - 1] == '\n' || line[logical_len - 1] == '\r')) {
    logical_len--;
  }
  line[logical_len] = '\0';

  // Update shared targets on valid parse
  CommandPacket pkt;
  if (logical_len > 0 && parse_command_line(line, &pkt) == 0) {
    shared_set_target_rpm(pkt.target_rpm_l, pkt.target_rpm_r);
    shared_mark_command_received();
  }
}

static void process_next_uart_event(TickType_t wait_ticks) {
  uart_event_t event;
  if (xQueueReceive(uart_event_queue, &event, wait_ticks) != pdTRUE) {
    return;
  }

  // Dispatch by event type
  switch (event.type) {
  case UART_PATTERN_DET:
    process_pattern_event();
    break;
  case UART_DATA:
    break;
  case UART_FIFO_OVF:
  case UART_BUFFER_FULL:
    ESP_LOGW(TAG, "UART overflow, flushing input");
    ESP_ERROR_CHECK(uart_flush_input(UART_NUM));
    xQueueReset(uart_event_queue);
    break;
  default:
    break;
  }
}

static void communication_task_entry(void *arg) {
  (void)arg;
  while (1) {
    process_next_uart_event(portMAX_DELAY);
  }
}

esp_err_t communication_init(void) {
  uart_config_t uart_config = {
      .baud_rate = UART_BAUD_RATE,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
  };

  ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
  ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN,
                               UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

  // Install driver, event queue, and newline pattern detection
  ESP_ERROR_CHECK(uart_driver_install(UART_NUM, UART_RX_BUFFER_SIZE,
                                      UART_TX_BUFFER_SIZE, UART_EVENT_QUEUE_LEN,
                                      &uart_event_queue, 0));
  ESP_ERROR_CHECK(
      uart_enable_pattern_det_baud_intr(UART_NUM, '\n', 1, 9, 0, 0));
  ESP_ERROR_CHECK(uart_pattern_queue_reset(UART_NUM, UART_EVENT_QUEUE_LEN));

  // Start communication task
  TaskHandle_t task_handle = NULL;
  if (xTaskCreate(communication_task_entry, "comm_task", COMM_TASK_STACK_WORDS,
                  NULL, COMM_TASK_PRIORITY, &task_handle) != pdPASS) {
    ESP_LOGE(TAG, "failed to create communication task");
    return ESP_FAIL;
  }

  ESP_LOGI(TAG, "Communication initialized at %d baud", UART_BAUD_RATE);
  return ESP_OK;
}

void send_telemetry(float rpm_l, float rpm_r, float pot_angle) {
  // Format: T,rpm_l,rpm_r,pot_angle
  char buf[64];
  int len =
      snprintf(buf, sizeof(buf), "T,%.3f,%.3f,%.3f", rpm_l, rpm_r, pot_angle);

  uint16_t crc = crc16_modbus((const uint8_t *)buf, len);
  len += snprintf(buf + len, sizeof(buf) - len, ",%04X\n", crc);

  uart_write_bytes(UART_NUM, buf, len);
}
