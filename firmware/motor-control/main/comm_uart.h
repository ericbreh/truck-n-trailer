#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  uint16_t seq;
  int64_t timestamp_ms;
  float target_rpm_l;
  float target_rpm_r;
  uint16_t ttl_ms;
  bool valid;
} CommandPacket;

void comm_uart_init(void);

int comm_uart_read_packet(CommandPacket *pkt);

int64_t comm_uart_get_command_age_ms(void);

void comm_uart_reset_sequence_guard(void);
