#pragma once

#include <stdint.h>

typedef struct {
  float target_rpm_l;
  float target_rpm_r;
} CommandPacket;

void comm_uart_init(void);

int comm_uart_read_packet(CommandPacket *pkt);

int64_t comm_uart_get_command_age_ms(void);
