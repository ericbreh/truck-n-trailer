#include "comm_uart.h"
#include "config.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#define UART_NUM UART_NUM_0

#define CMD_START "CMD,"

static const char *TAG = "comm_uart";

static int64_t last_cmd_rx_local_ms = -1;
static uint16_t last_cmd_seq = 0;
static bool has_last_seq = false;

static char rx_line[UART_COMMAND_MAX_LEN];
static size_t rx_line_len = 0;
static bool rx_overflow = false;

static uint16_t crc16_modbus(const uint8_t *data, size_t len) {
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

void comm_uart_init(void) {
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
  ESP_ERROR_CHECK(uart_driver_install(UART_NUM, UART_RX_BUFFER_SIZE,
                                      UART_TX_BUFFER_SIZE, 0, NULL, 0));
  ESP_LOGI(TAG, "UART initialized at %d baud", UART_BAUD_RATE);
}

static bool parse_u16_token(const char *token, uint16_t *out) {
  char *end = NULL;
  errno = 0;
  unsigned long value = strtoul(token, &end, 10);
  if (errno != 0 || end == token || *end != '\0' || value > UINT16_MAX) {
    return false;
  }
  *out = (uint16_t)value;
  return true;
}

static bool parse_i64_token(const char *token, int64_t *out) {
  char *end = NULL;
  errno = 0;
  long long value = strtoll(token, &end, 10);
  if (errno != 0 || end == token || *end != '\0') {
    return false;
  }
  *out = (int64_t)value;
  return true;
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

static bool is_newer_seq(uint16_t seq) {
  if (!has_last_seq) {
    return true;
  }
  int16_t delta = (int16_t)(seq - last_cmd_seq);
  return delta > 0;
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
  char *seq_token = strtok_r(body, ",", &saveptr);
  char *ts_token = strtok_r(NULL, ",", &saveptr);
  char *rpm_l_token = strtok_r(NULL, ",", &saveptr);
  char *rpm_r_token = strtok_r(NULL, ",", &saveptr);
  char *ttl_token = strtok_r(NULL, ",", &saveptr);
  char *extra_token = strtok_r(NULL, ",", &saveptr);

  if (seq_token == NULL || ts_token == NULL || rpm_l_token == NULL ||
      rpm_r_token == NULL || ttl_token == NULL || extra_token != NULL) {
    return -1;
  }

  uint16_t seq = 0;
  uint16_t ttl_ms = 0;
  int64_t timestamp_ms = 0;
  float rpm_l = 0.0f;
  float rpm_r = 0.0f;

  if (!parse_u16_token(seq_token, &seq) ||
      !parse_i64_token(ts_token, &timestamp_ms) ||
      !parse_float_token(rpm_l_token, &rpm_l) ||
      !parse_float_token(rpm_r_token, &rpm_r) ||
      !parse_u16_token(ttl_token, &ttl_ms)) {
    return -1;
  }

  if (!is_newer_seq(seq)) {
    return -1;
  }

  pkt->seq = seq;
  pkt->timestamp_ms = timestamp_ms;
  pkt->target_rpm_l = rpm_l;
  pkt->target_rpm_r = rpm_r;
  pkt->ttl_ms = ttl_ms;
  pkt->valid = true;

  last_cmd_seq = seq;
  has_last_seq = true;
  last_cmd_rx_local_ms = esp_timer_get_time() / 1000;
  return 0;
}

int comm_uart_read_packet(CommandPacket *pkt) {
  uint8_t buf[UART_COMMAND_MAX_LEN];
  int len = uart_read_bytes(UART_NUM, buf, sizeof(buf), pdMS_TO_TICKS(0));
  if (len <= 0) {
    return -1;
  }

  bool got_packet = false;

  for (int i = 0; i < len; i++) {
    char c = (char)buf[i];

    if (c == '\n' || c == '\r') {
      if (!rx_overflow && rx_line_len > 0) {
        rx_line[rx_line_len] = '\0';
        if (parse_command_line(rx_line, pkt) == 0) {
          got_packet = true;
        }
      }
      rx_line_len = 0;
      rx_overflow = false;
      continue;
    }

    if (rx_overflow) {
      continue;
    }

    if (rx_line_len >= (sizeof(rx_line) - 1)) {
      rx_overflow = true;
      rx_line_len = 0;
      continue;
    }

    rx_line[rx_line_len++] = c;
  }

  return got_packet ? 0 : -1;
}

int64_t comm_uart_get_command_age_ms(void) {
  if (last_cmd_rx_local_ms < 0) {
    return INT64_MAX;
  }
  return (esp_timer_get_time() / 1000) - last_cmd_rx_local_ms;
}

void comm_uart_reset_sequence_guard(void) {
  has_last_seq = false;
}
