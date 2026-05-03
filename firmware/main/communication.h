#pragma once

#include "esp_err.h"
#include <stdint.h>

uint16_t crc16_modbus(const uint8_t *data, size_t len);

esp_err_t communication_init(void);
void send_telemetry(float rpm_l, float rpm_r, float pot_angle);
